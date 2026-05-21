#include "mdfly/simulator.h"
#include "json.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>

using json = nlohmann::json;

namespace mdfly {

void MDFly::load_config(const std::string& config_path) {
    std::ifstream f(config_path);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open config file: " + config_path);
    }
    json j;
    f >> j;

    dt = j.value("dt", 0.01);
    rho = j.value("rho", 1.225);
    g = j.value("g", 9.81);

    // Actuation config
    std::vector<std::string> act_dynamics = j["actuation"]["dynamics"].get<std::vector<std::string>>();
    std::vector<std::string> act_inputs = j["actuation"]["inputs"].get<std::vector<std::string>>();
    actuation = Actuation(model_inputs, act_inputs, act_dynamics);

    // Read variables config
    for (const auto& var_j : j["variables"]) {
        std::string name = var_j["name"];
        std::string unit = var_j.value("unit", "");
        std::string label = var_j.value("label", "");

        bool convert_to_rad = var_j.value("convert_to_radians", false);

        std::unique_ptr<Variable> v;

        if (std::find(actuator_states.begin(), actuator_states.end(), name) != actuator_states.end()) {
            auto cv = std::make_unique<ControlVariable>(name, unit, label);
            cv->disabled = var_j.value("disabled", false);
            cv->order = var_j.value("order", 0);
            cv->tau = var_j.value("tau", 0.0);
            cv->omega_0 = var_j.value("omega_0", 0.0);
            cv->zeta = var_j.value("zeta", 0.0);
            
            if (var_j.contains("dot_max")) {
                cv->dot_max = var_j["dot_max"];
                cv->has_dot_max = true;
            }

            // Set transfer function coefficients
            if (cv->order == 1) {
                cv->coefs[0][0] = -1.0 / cv->tau;
                cv->coefs[0][1] = 0.0;
                cv->coefs[0][2] = 1.0 / cv->tau;
                cv->coefs[1][0] = 0.0;
                cv->coefs[1][1] = 0.0;
                cv->coefs[1][2] = 0.0;
            } else if (cv->order == 2) {
                cv->coefs[0][0] = 0.0;
                cv->coefs[0][1] = 1.0;
                cv->coefs[0][2] = 0.0;
                cv->coefs[1][0] = -cv->omega_0 * cv->omega_0;
                cv->coefs[1][1] = -2.0 * cv->zeta * cv->omega_0;
                cv->coefs[1][2] = cv->omega_0 * cv->omega_0;
            }

            v = std::move(cv);
        } else if (name.find("energy") != std::string::npos) {
            energy_states.push_back(name);
            v = std::make_unique<EnergyVariable>(name, params.mass, g, I, unit, label);
        } else if (name.find("momentum") != std::string::npos) {
            momentum_states.push_back(name);
            v = std::make_unique<MomentumVariable>(name, params.mass, I, unit, label);
        } else {
            v = std::make_unique<Variable>(name, unit, label);
        }

        v->convert_to_radians = convert_to_rad;
        
        // Min / Max constraints
        auto parse_double_and_convert = [&](const json& obj, const std::string& key, double& target, bool& flag) {
            if (obj.contains(key)) {
                target = obj[key].get<double>();
                if (convert_to_rad) {
                    target = target * M_PI / 180.0;
                }
                flag = true;
            }
        };

        parse_double_and_convert(var_j, "value_min", v->value_min, v->has_value_min);
        parse_double_and_convert(var_j, "value_max", v->value_max, v->has_value_max);
        parse_double_and_convert(var_j, "init_min", v->init_min, v->has_init_min);
        parse_double_and_convert(var_j, "init_max", v->init_max, v->has_init_max);
        parse_double_and_convert(var_j, "constraint_min", v->constraint_min, v->has_constraint_min);
        parse_double_and_convert(var_j, "constraint_max", v->constraint_max, v->has_constraint_max);

        v->wrap = var_j.value("wrap", false);

        state[name] = std::move(v);
    }

    // Add disabled control variables if not already present
    for (const auto& in_name : model_inputs) {
        if (state.find(in_name) == state.end()) {
            auto cv = std::make_unique<ControlVariable>(in_name);
            cv->disabled = true;
            state[in_name] = std::move(cv);
        }
    }

    // Populate wind components
    std::vector<Variable*> wind_comps;
    for (const auto& w_name : {"wind_n", "wind_e", "wind_d"}) {
        if (state.find(w_name) != state.end()) {
            wind_comps.push_back(state[w_name].get());
        }
    }

    bool wind_turb = j.value("turbulence", false);
    double wind_mag_min = j.value("wind_magnitude_min", 0.0);
    double wind_mag_max = j.value("wind_magnitude_max", 0.0);
    std::string intensity = j.value("turbulence_intensity", "light");

    wind = Wind(wind_turb, wind_mag_min, wind_mag_max, params.b, intensity, dt);
    wind.components = wind_comps;

    // Register variables with actuation
    for (auto& pair : state) {
        if (std::find(actuator_states.begin(), actuator_states.end(), pair.first) != actuator_states.end()) {
            actuation.add_state(dynamic_cast<ControlVariable*>(pair.second.get()));
        }
    }

    actuation.finalize();

    // Map energy variable requirements
    for (const auto& e_name : energy_states) {
        auto* ev = dynamic_cast<EnergyVariable*>(state[e_name].get());
        for (const auto& req : ev->required_variables) {
            ev->add_requirement(req, state[req].get());
        }
    }

    // Map momentum variable requirements
    for (const auto& m_name : momentum_states) {
        auto* mv = dynamic_cast<MomentumVariable*>(state[m_name].get());
        // Momentum requires velocity, omega, and quaternion components
        for (const auto& req : {"velocity_u", "velocity_v", "velocity_w", "omega_p", "omega_q", "omega_r"}) {
            mv->add_requirement(req, state[req].get());
        }
        // Also map quaternion variables if it's an inertial frame calculation
        if (m_name.find("_n") != std::string::npos || m_name.find("_e") != std::string::npos || m_name.find("_d") != std::string::npos) {
            // We will dynamically feed the quaternion values. Wait! We can map variables "q0", "q1", "q2", "q3" by creating dummy variables in state to hold quaternion values
        }
    }

    // Create dummy state variables to hold quaternion components for momentum calculations
    for (int i = 0; i < 4; ++i) {
        std::string qname = "q" + std::to_string(i);
        state[qname] = std::make_unique<Variable>(qname);
    }
    for (const auto& m_name : momentum_states) {
        auto* mv = dynamic_cast<MomentumVariable*>(state[m_name].get());
        for (int i = 0; i < 4; ++i) {
            std::string qname = "q" + std::to_string(i);
            mv->add_requirement(qname, state[qname].get());
        }
    }

    // Determine attitude states with constraints
    for (const auto& att_name : attitude_states) {
        const auto* v = state[att_name].get();
        if (v->has_constraint_min || v->has_constraint_max || v->has_value_min || v->has_value_max) {
            attitude_states_with_constraints.push_back(att_name);
        }
    }
}

void MDFly::load_parameters(const std::string& param_path) {
    std::ifstream f(param_path);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open parameter file: " + param_path);
    }
    json j;
    f >> j;

    params.mass = j["mass"].get<double>();
    params.Jx = j["Jx"].get<double>();
    params.Jy = j["Jy"].get<double>();
    params.Jz = j["Jz"].get<double>();
    params.Jxz = j["Jxz"].get<double>();

    params.S_wing = j["S_wing"].get<double>();
    params.b = j["b"].get<double>();
    params.c = j["c"].get<double>();
    params.ar = params.b * params.b / params.S_wing;

    params.S_prop = j["S_prop"].get<double>();
    params.C_prop = j["C_prop"].get<double>();
    params.k_motor = j["k_motor"].get<double>();
    params.k_T_P = j["k_T_P"].get<double>();
    params.k_Omega = j["k_Omega"].get<double>();

    params.C_L_0 = j["C_L_0"].get<double>();
    params.C_L_alpha = j["C_L_alpha"].get<double>();
    params.C_L_q = j["C_L_q"].get<double>();
    params.C_L_delta_e = j["C_L_delta_e"].get<double>();

    params.C_D_0 = j["C_D_0"].get<double>();
    params.C_D_alpha1 = j["C_D_alpha1"].get<double>();
    params.C_D_alpha2 = j["C_D_alpha2"].get<double>();
    params.C_D_beta1 = j["C_D_beta1"].get<double>();
    params.C_D_beta2 = j["C_D_beta2"].get<double>();
    params.C_D_delta_e = j["C_D_delta_e"].get<double>();
    params.C_D_p = j["C_D_p"].get<double>();
    params.C_D_q = j["C_D_q"].get<double>();

    params.C_Y_0 = j["C_Y_0"].get<double>();
    params.C_Y_beta = j["C_Y_beta"].get<double>();
    params.C_Y_delta_a = j["C_Y_delta_a"].get<double>();
    params.C_Y_delta_r = j["C_Y_delta_r"].get<double>();
    params.C_Y_p = j["C_Y_p"].get<double>();
    params.C_Y_r = j["C_Y_r"].get<double>();

    params.C_l_0 = j["C_l_0"].get<double>();
    params.C_l_beta = j["C_l_beta"].get<double>();
    params.C_l_delta_a = j["C_l_delta_a"].get<double>();
    params.C_l_delta_r = j["C_l_delta_r"].get<double>();
    params.C_l_p = j["C_l_p"].get<double>();
    params.C_l_r = j["C_l_r"].get<double>();

    params.C_m_0 = j["C_m_0"].get<double>();
    params.C_m_alpha = j["C_m_alpha"].get<double>();
    params.C_m_q = j["C_m_q"].get<double>();
    params.C_m_delta_e = j["C_m_delta_e"].get<double>();
    params.C_m_fp = j["C_m_fp"].get<double>();

    params.C_n_0 = j["C_n_0"].get<double>();
    params.C_n_beta = j["C_n_beta"].get<double>();
    params.C_n_delta_a = j["C_n_delta_a"].get<double>();
    params.C_n_delta_r = j["C_n_delta_r"].get<double>();
    params.C_n_p = j["C_n_p"].get<double>();
    params.C_n_r = j["C_n_r"].get<double>();

    params.M = j["M"].get<double>();
    params.a_0 = j["a_0"].get<double>();
    params.Oswald_e = j["e"].get<double>();

    // Expanded / Missing aerodynamic coefficients: Load if present, else default to 0
    params.C_L_alpha_dot = j.value("C_L_alpha_dot", 0.0);
    params.C_L_delta_a = j.value("C_L_delta_a", 0.0);
    params.C_L_delta_r = j.value("C_L_delta_r", 0.0);

    params.C_D_delta_a = j.value("C_D_delta_a", 0.0);
    params.C_D_delta_r = j.value("C_D_delta_r", 0.0);
    params.C_D_alpha_dot = j.value("C_D_alpha_dot", 0.0);

    params.C_Y_q = j.value("C_Y_q", 0.0);
    params.C_Y_delta_e = j.value("C_Y_delta_e", 0.0);

    params.C_l_q = j.value("C_l_q", 0.0);
    params.C_l_delta_e = j.value("C_l_delta_e", 0.0);

    params.C_m_beta = j.value("C_m_beta", 0.0);
    params.C_m_delta_a = j.value("C_m_delta_a", 0.0);
    params.C_m_delta_r = j.value("C_m_delta_r", 0.0);
    params.C_m_alpha_dot = j.value("C_m_alpha_dot", 0.0);

    params.C_n_q = j.value("C_n_q", 0.0);
    params.C_n_delta_e = j.value("C_n_delta_e", 0.0);

    // Build I matrix
    I[0][0] = params.Jx;
    I[0][1] = 0.0;
    I[0][2] = -params.Jxz;

    I[1][0] = 0.0;
    I[1][1] = params.Jy;
    I[1][2] = 0.0;

    I[2][0] = -params.Jxz;
    I[2][1] = 0.0;
    I[2][2] = params.Jz;

    calculate_gammas();
}

void MDFly::calculate_gammas() {
    gammas[0] = I[0][0] * I[2][2] - I[0][2] * I[0][2];
    gammas[1] = (std::abs(I[0][2]) * (I[0][0] - I[1][1] + I[2][2])) / gammas[0];
    gammas[2] = (I[2][2] * (I[2][2] - I[1][1]) + I[0][2] * I[0][2]) / gammas[0];
    gammas[3] = I[2][2] / gammas[0];
    gammas[4] = std::abs(I[0][2]) / gammas[0];
    gammas[5] = (I[2][2] - I[0][0]) / I[1][1];
    gammas[6] = std::abs(I[0][2]) / I[1][1];
    gammas[7] = ((I[0][0] - I[1][1]) * I[0][0] + I[0][2] * I[0][2]) / gammas[0];
    gammas[8] = I[0][0] / gammas[0];
}

void MDFly::seed(unsigned int seed_val) {
    unsigned int idx = 0;
    for (auto& pair : state) {
        pair.second->seed(seed_val + idx);
        idx++;
    }
    wind.seed(seed_val);
}

void MDFly::reset(const std::map<std::string, double>& state_init) {
    cur_sim_step = 0;

    // Reset standard state variables (excluding Va, alpha, beta, attitude, wind, actuators, and energy/momentum)
    for (auto& pair : state) {
        const std::string& name = pair.first;
        Variable* var = pair.second.get();
        if (name == "Va" || name == "alpha" || name == "beta" || name == "attitude" ||
            name.find("wind") != std::string::npos || name.find("energy") != std::string::npos ||
            name.find("momentum") != std::string::npos || name.find("q") == 0 ||
            std::find(actuator_states.begin(), actuator_states.end(), name) != actuator_states.end()) {
            continue;
        }

        auto it = state_init.find(name);
        if (it != state_init.end()) {
            var->reset(it->second);
        } else {
            var->reset();
        }
    }

    actuation.reset(state_init);

    // Reset wind
    std::vector<double> wind_init;
    auto it_w = state_init.find("wind");
    if (it_w != state_init.end()) {
        wind_init.push_back(it_w->second);
    } else {
        auto it_wn = state_init.find("wind_n");
        auto it_we = state_init.find("wind_e");
        auto it_wd = state_init.find("wind_d");
        if (it_wn != state_init.end() && it_we != state_init.end() && it_wd != state_init.end()) {
            wind_init = {it_wn->second, it_we->second, it_wd->second};
        }
    }

    wind.reset(wind_init);

    // Initial attitude
    std::array<double, 3> Theta = {
        state["roll"]->value,
        state["pitch"]->value,
        state["yaw"]->value
    };
    attitude.reset(Theta);

    // Update quaternion variables for momentum calculation
    for (int i = 0; i < 4; ++i) {
        state["q" + std::to_string(i)]->set_value(attitude.quaternion[i], false);
    }

    // Initial airspeed factors
    std::array<double, 3> vel = {
        state["velocity_u"]->value,
        state["velocity_v"]->value,
        state["velocity_w"]->value
    };
    std::array<double, 3> airspeed = calculate_airspeed_factors(attitude.quaternion, vel);
    state["Va"]->reset(airspeed[0]);
    state["alpha"]->reset(airspeed[1]);
    state["beta"]->reset(airspeed[2]);

    // Initial energy and momentum variables
    for (const auto& e_name : energy_states) {
        auto* ev = dynamic_cast<EnergyVariable*>(state[e_name].get());
        ev->reset(ev->calculate_value());
    }
    for (const auto& m_name : momentum_states) {
        auto* mv = dynamic_cast<MomentumVariable*>(state[m_name].get());
        mv->reset(mv->calculate_value());
    }
}

bool MDFly::step(const std::vector<double>& commands, std::string& term_reason) {
    try {
        last_gear_cmd = (commands.size() >= 4) ? commands[3] : 1.0;
        std::vector<double> act_commands = commands;
        if (act_commands.size() > actuation.inputs.size()) {
            act_commands.resize(actuation.inputs.size());
        }
        std::vector<double> control_inputs = actuation.set_and_constrain_commands(act_commands);

        // Pack ODE state vector
        // 4 (quaternion) + 3 (omega) + 3 (position) + 3 (velocity) + 1 (motor_omega) + 1 (gear_deploy) + 2 * dynamics.size() (actuators)
        std::vector<double> y0;
        y0.reserve(15 + 2 * actuation.dynamics.size());
        
        y0.push_back(attitude.quaternion[0]);
        y0.push_back(attitude.quaternion[1]);
        y0.push_back(attitude.quaternion[2]);
        y0.push_back(attitude.quaternion[3]);

        for (const auto& w : {"omega_p", "omega_q", "omega_r"}) y0.push_back(state[w]->value);
        for (const auto& p : {"position_n", "position_e", "position_d"}) y0.push_back(state[p]->value);
        for (const auto& v : {"velocity_u", "velocity_v", "velocity_w"}) y0.push_back(state[v]->value);
        
        y0.push_back(state["motor_omega"]->value);
        y0.push_back(state["gear_deploy"]->value);
        
        std::vector<double> act_y = actuation.get_values();
        y0.insert(y0.end(), act_y.begin(), act_y.end());

        // Perform fixed-step RK4 integration over dt
        // Since we integration over dt, we define the derivatives:
        // k1 = dt * f(0, y0)
        // k2 = dt * f(dt/2, y0 + k1/2)
        // k3 = dt * f(dt/2, y0 + k2/2)
        // k4 = dt * f(dt, y0 + k3)
        // y_next = y0 + (k1 + 2*k2 + 2*k3 + k4)/6
        
        std::vector<double> k1 = ode_dynamics(0.0, y0, control_inputs);
        
        std::vector<double> y_k2(y0.size());
        for (size_t i = 0; i < y0.size(); ++i) y_k2[i] = y0[i] + 0.5 * dt * k1[i];
        std::vector<double> k2 = ode_dynamics(0.5 * dt, y_k2, control_inputs);

        std::vector<double> y_k3(y0.size());
        for (size_t i = 0; i < y0.size(); ++i) y_k3[i] = y0[i] + 0.5 * dt * k2[i];
        std::vector<double> k3 = ode_dynamics(0.5 * dt, y_k3, control_inputs);

        std::vector<double> y_k4(y0.size());
        for (size_t i = 0; i < y0.size(); ++i) y_k4[i] = y0[i] + dt * k3[i];
        std::vector<double> k4 = ode_dynamics(dt, y_k4, control_inputs);

        std::vector<double> y_next(y0.size());
        for (size_t i = 0; i < y0.size(); ++i) {
            y_next[i] = y0[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }

        // Set states from the integrated ODE solution
        set_states_from_ode_solution(y_next, true);

        // Update attitude quaternion components in state for momentum calculation
        for (int i = 0; i < 4; ++i) {
            state["q" + std::to_string(i)]->set_value(attitude.quaternion[i], false);
        }

        // Calculate Va, alpha, beta
        std::array<double, 3> Theta = {
            state["roll"]->value,
            state["pitch"]->value,
            state["yaw"]->value
        };
        std::array<double, 3> vel = {
            state["velocity_u"]->value,
            state["velocity_v"]->value,
            state["velocity_w"]->value
        };
        std::array<double, 3> airspeed = calculate_airspeed_factors(attitude.quaternion, vel);
        state["Va"]->set_value(airspeed[0]);
        state["alpha"]->set_value(airspeed[1]);
        state["beta"]->set_value(airspeed[2]);

        // Update derived variables
        for (const auto& e_name : energy_states) {
            auto* ev = dynamic_cast<EnergyVariable*>(state[e_name].get());
            ev->set_value(ev->calculate_value(), true);
        }
        for (const auto& m_name : momentum_states) {
            auto* mv = dynamic_cast<MomentumVariable*>(state[m_name].get());
            mv->set_value(mv->calculate_value(), true);
        }

        // Update target bounding boxes
        std::array<double, 3> host_pos = {
            state["position_n"]->value,
            state["position_e"]->value,
            state["position_d"]->value
        };
        for (size_t i = 0; i < targets.size(); ++i) {
            target_bboxes[i] = camera.project_target(host_pos, attitude.quaternion, targets[i]);
        }

        // Advance wind step
        wind.step(cur_sim_step);
        cur_sim_step++;

        return true;
    } catch (const std::exception& e) {
        term_reason = e.what();
        return false;
    }
}

std::vector<double> MDFly::get_states_vector(const std::vector<std::string>& states_list, const std::string& attribute) const {
    std::vector<double> vals;
    vals.reserve(states_list.size());
    for (const auto& name : states_list) {
        vals.push_back(state.at(name)->value);
    }
    return vals;
}

std::vector<double> MDFly::ode_dynamics(double t, const std::vector<double>& y, const std::vector<double>& control_sp) {
    if (t > 0.0) {
        set_states_from_ode_solution(y, false);
    }

    double q_norm = std::sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2] + y[3] * y[3]);
    std::array<double, 4> att_q = {y[0] / q_norm, y[1] / q_norm, y[2] / q_norm, y[3] / q_norm};
    std::array<double, 3> omega = {
        state["omega_p"]->value,
        state["omega_q"]->value,
        state["omega_r"]->value
    };
    std::array<double, 3> vel = {
        state["velocity_u"]->value,
        state["velocity_v"]->value,
        state["velocity_w"]->value
    };
    
    std::vector<double> ctrl_vals = get_states_vector(model_inputs);
    std::array<double, 4> controls = {ctrl_vals[0], ctrl_vals[1], ctrl_vals[2], ctrl_vals[3]};

    // Calculate forces and moments
    auto forces_moments = forces(att_q, omega, vel, controls);
    std::array<double, 3> f = forces_moments.first;
    std::array<double, 3> tau = forces_moments.second;

    std::vector<double> dydt;
    dydt.reserve(y.size());

    // 1. Attitude Quaternion dot
    double p = omega[0];
    double q = omega[1];
    double r = omega[2];
    dydt.push_back(0.5 * (-p * att_q[1] - q * att_q[2] - r * att_q[3]));
    dydt.push_back(0.5 * (p * att_q[0] + r * att_q[2] - q * att_q[3]));
    dydt.push_back(0.5 * (q * att_q[0] - r * att_q[1] + p * att_q[3]));
    dydt.push_back(0.5 * (r * att_q[0] + q * att_q[1] - p * att_q[2]));

    // 2. Omega dot
    dydt.push_back(gammas[1] * p * q - gammas[2] * q * r + gammas[3] * tau[0] + gammas[4] * tau[2]);
    dydt.push_back(gammas[5] * p * r - gammas[6] * (p * p - r * r) + tau[1] / I[1][1]);
    dydt.push_back(gammas[7] * p * q - gammas[1] * q * r + gammas[4] * tau[0] + gammas[8] * tau[2]);

    // 3. Position dot (Inertial Velocity = R_b/v * V_body)
    std::array<std::array<double, 3>, 3> Rvb = rot_b_v(att_q); // Rvb is vehicle-to-body
    dydt.push_back(Rvb[0][0] * vel[0] + Rvb[1][0] * vel[1] + Rvb[2][0] * vel[2]);
    dydt.push_back(Rvb[0][1] * vel[0] + Rvb[1][1] * vel[1] + Rvb[2][1] * vel[2]);
    dydt.push_back(Rvb[0][2] * vel[0] + Rvb[1][2] * vel[1] + Rvb[2][2] * vel[2]);

    // 4. Linear Velocity dot (Acceleration in body frame)
    dydt.push_back(r * vel[1] - q * vel[2] + f[0] / params.mass);
    dydt.push_back(p * vel[2] - r * vel[0] + f[1] / params.mass);
    dydt.push_back(q * vel[0] - p * vel[1] + f[2] / params.mass);

    // 5. Motor speed dot
    dydt.push_back(last_motor_omega_dot);

    // 6. Landing gear deploy dot
    double gear_dot = (last_gear_cmd - y[14]) / 1.0; // tau = 1.0s
    dydt.push_back(gear_dot);

    // 7. Actuator dynamics
    std::vector<double> act_rhs = actuation.rhs(control_sp);
    dydt.insert(dydt.end(), act_rhs.begin(), act_rhs.end());

    return dydt;
}

std::pair<std::array<double, 3>, std::array<double, 3>> MDFly::forces(
    const std::array<double, 4>& att_q,
    const std::array<double, 3>& omega,
    const std::array<double, 3>& vel,
    const std::array<double, 4>& controls
) {
    double elevator = controls[0];
    double aileron = controls[1];
    double rudder = controls[2];
    double throttle = controls[3];

    double p = omega[0];
    double q = omega[1];
    double r = omega[2];

    if (wind.turbulence) {
        p -= wind.current_angular_turbulence[0];
        q -= wind.current_angular_turbulence[1];
        r -= wind.current_angular_turbulence[2];
    }

    std::array<double, 3> airspeed = calculate_airspeed_factors(att_q, vel);
    double Va = state["Va"]->apply_conditions(airspeed[0]);
    double alpha = state["alpha"]->apply_conditions(airspeed[1]);
    double beta = state["beta"]->apply_conditions(airspeed[2]);

    double pre_fac = 0.5 * rho * Va * Va * params.S_wing;

    // Gravity in body frame (rotated using R_v/b third column)
    double e0 = att_q[0];
    double e1 = att_q[1];
    double e2 = att_q[2];
    double e3 = att_q[3];
    std::array<double, 3> fg_b = {
        params.mass * g * 2.0 * (e1 * e3 - e2 * e0),
        params.mass * g * 2.0 * (e2 * e3 + e1 * e0),
        params.mass * g * (e3 * e3 + e0 * e0 - e1 * e1 - e2 * e2)
    };

    // Sigmoid function for stall model
    double M = params.M;
    double a_0 = params.a_0;
    double exp_neg = std::exp(-M * (alpha - a_0));
    double exp_pos = std::exp(M * (alpha + a_0));
    double sigma = (1.0 + exp_neg + exp_pos) / ((1.0 + exp_neg) * (1.0 + exp_pos));

    // Lift
    double C_L_alpha_lin = params.C_L_0 + params.C_L_alpha * alpha;
    double C_L_alpha_stall = 2.0 * ((alpha >= 0.0) ? 1.0 : -1.0) * std::pow(std::sin(alpha), 2) * std::cos(alpha);
    double C_L_alpha = (1.0 - sigma) * C_L_alpha_lin + sigma * C_L_alpha_stall;
    
    // Exact alpha_dot is assumed 0 for fixed-coefficients, but we support C_L_alpha_dot if computed.
    double alpha_dot = 0.0; 

    double C_L = C_L_alpha + params.C_L_q * (params.c / (2.0 * Va)) * q + params.C_L_delta_e * elevator
                 + params.C_L_alpha_dot * (params.c / (2.0 * Va)) * alpha_dot
                 + params.C_L_delta_a * aileron + params.C_L_delta_r * rudder;
    double f_lift_s = pre_fac * C_L;

    // Drag
    double C_D_alpha_lin = params.C_D_p + std::pow(params.C_L_0 + params.C_L_alpha * alpha, 2) / (M_PI * params.Oswald_e * params.ar);
    double C_D_alpha_stall = 2.0 * ((alpha >= 0.0) ? 1.0 : -1.0) * std::pow(std::sin(alpha), 3);
    double C_D_alpha = params.C_D_p + (1.0 - sigma) * (C_D_alpha_lin - params.C_D_p) + sigma * C_D_alpha_stall;
    double C_D_beta = params.C_D_beta1 * beta + params.C_D_beta2 * beta * beta;
    
    double gear_deploy_val = state["gear_deploy"]->value;
    double C_D = C_D_alpha + C_D_beta + params.C_D_q * (params.c / (2.0 * Va)) * q + params.C_D_delta_e * elevator * elevator
                 + params.C_D_delta_a * aileron + params.C_D_delta_r * rudder
                 + params.C_D_alpha_dot * (params.c / (2.0 * Va)) * alpha_dot
                 + 0.035 * gear_deploy_val;
    double f_drag_s = pre_fac * C_D;

    // Pitching Moment (damping non-dimensionalized by wingspan b, keeping PyFly bug/convention for trajectory matching)
    double C_m_alpha_lin = params.C_m_0 + params.C_m_alpha * alpha;
    double C_m_alpha_stall = params.C_m_fp * ((alpha >= 0.0) ? 1.0 : -1.0) * std::pow(std::sin(alpha), 2);
    double C_m_a = (1.0 - sigma) * C_m_alpha_lin + sigma * C_m_alpha_stall;
    
    double C_m = C_m_a + params.C_m_q * (params.b / (2.0 * Va)) * q + params.C_m_delta_e * elevator
                 + params.C_m_beta * beta + params.C_m_delta_a * aileron + params.C_m_delta_r * rudder
                 + params.C_m_alpha_dot * (params.c / (2.0 * Va)) * alpha_dot;
    double m = pre_fac * params.c * C_m;

    // Sideforce
    double C_Y = params.C_Y_0 + params.C_Y_beta * beta + params.C_Y_p * (params.b / (2.0 * Va)) * p 
                 + params.C_Y_r * (params.b / (2.0 * Va)) * r + params.C_Y_delta_a * aileron 
                 + params.C_Y_delta_r * rudder + params.C_Y_q * (params.c / (2.0 * Va)) * q 
                 + params.C_Y_delta_e * elevator;
    double f_y = pre_fac * C_Y;

    // Rolling Moment
    double C_l = params.C_l_0 + params.C_l_beta * beta + params.C_l_p * (params.b / (2.0 * Va)) * p 
                 + params.C_l_r * (params.b / (2.0 * Va)) * r + params.C_l_delta_a * aileron 
                 + params.C_l_delta_r * rudder + params.C_l_q * (params.c / (2.0 * Va)) * q 
                 + params.C_l_delta_e * elevator;
    double l = pre_fac * params.b * C_l;

    // Yawing Moment
    double C_n = params.C_n_0 + params.C_n_beta * beta + params.C_n_p * (params.b / (2.0 * Va)) * p 
                 + params.C_n_r * (params.b / (2.0 * Va)) * r + params.C_n_delta_a * aileron 
                 + params.C_n_delta_r * rudder + params.C_n_q * (params.c / (2.0 * Va)) * q 
                 + params.C_n_delta_e * elevator;
    double n = pre_fac * params.b * C_n;

    // Rotate stability/wind frame forces to body frame
    // R_stab/body = rot_b_v with phi=0, th=alpha, psi=beta
    double c_a = std::cos(alpha);
    double s_a = std::sin(alpha);
    double c_b = std::cos(beta);
    double s_b = std::sin(beta);

    double r_sb[3][3];
    r_sb[0][0] = c_a * c_b;
    r_sb[0][1] = c_a * s_b;
    r_sb[0][2] = -s_a;
    
    r_sb[1][0] = -s_b;
    r_sb[1][1] = c_b;
    r_sb[1][2] = 0.0;
    
    r_sb[2][0] = s_a * c_b;
    r_sb[2][1] = s_a * s_b;
    r_sb[2][2] = c_a;

    double f_drag_val = -f_drag_s;
    double f_y_val = f_y;
    double f_lift_val = -f_lift_s;

    std::array<double, 3> f_aero;
    f_aero[0] = r_sb[0][0] * f_drag_val + r_sb[0][1] * f_y_val + r_sb[0][2] * f_lift_val;
    f_aero[1] = r_sb[1][0] * f_drag_val + r_sb[1][1] * f_y_val + r_sb[1][2] * f_lift_val;
    f_aero[2] = r_sb[2][0] * f_drag_val + r_sb[2][1] * f_y_val + r_sb[2][2] * f_lift_val;

    std::array<double, 3> tau_aero = {l, m, n};

    // Propeller forces and moment
    double thrust = 0.0;
    double drag_torque = 0.0;
    double omega_dot = 0.0;
    double m_omega = state["motor_omega"]->value;

    motor.compute_dynamics(Va, m_omega, throttle, rho, thrust, drag_torque, omega_dot);
    last_motor_omega_dot = omega_dot;

    std::array<double, 3> f_prop = { thrust, 0.0, 0.0 };
    std::array<double, 3> tau_prop = { -drag_torque, 0.0, 0.0 };

    // Ground Contact Reaction Forces
    std::array<std::array<double, 3>, 3> Rvb = rot_b_v(att_q);
    double d = state["position_d"]->value;
    double F_normal_inertial = 0.0;
    double friction_x_inertial = 0.0;
    double friction_y_inertial = 0.0;

    if (d >= 0.0) {
        // Rvb is vehicle-to-body, so its transpose is body-to-vehicle Rbv
        double Rbv_0_0 = Rvb[0][0], Rbv_0_1 = Rvb[1][0], Rbv_0_2 = Rvb[2][0];
        double Rbv_1_0 = Rvb[0][1], Rbv_1_1 = Rvb[1][1], Rbv_1_2 = Rvb[2][1];
        double Rbv_2_0 = Rvb[0][2], Rbv_2_1 = Rvb[1][2], Rbv_2_2 = Rvb[2][2];

        double vel_d = Rbv_2_0*vel[0] + Rbv_2_1*vel[1] + Rbv_2_2*vel[2];
        double vel_n = Rbv_0_0*vel[0] + Rbv_0_1*vel[1] + Rbv_0_2*vel[2];
        double vel_e = Rbv_1_0*vel[0] + Rbv_1_1*vel[1] + Rbv_1_2*vel[2];

        double K_spring = (gear_deploy_val > 0.5) ? 3500.0 : 16000.0; // stiffer belly landing
        double C_damper = (gear_deploy_val > 0.5) ? 180.0 : 600.0;

        double F_spring = K_spring * d; // Corrected sign: pushes up when below ground
        double F_damper = C_damper * vel_d; // Corrected sign: resists downward velocity

        double F_normal = F_spring + F_damper;
        if (F_normal < 0.0) F_normal = 0.0;

        F_normal_inertial = -F_normal; // points up (negative D)

        double speed_horizontal = std::sqrt(vel_n*vel_n + vel_e*vel_e);
        if (speed_horizontal > 0.05) {
            double mu = 0.65; // belly landing friction
            if (gear_deploy_val > 0.5) {
                mu = 0.03; // rolling resistance
                if (last_gear_cmd < 0.1) {
                    mu = 0.35; // braking rolling resistance
                }
            }
            double F_friction = mu * F_normal;
            friction_x_inertial = -F_friction * (vel_n / speed_horizontal);
            friction_y_inertial = -F_friction * (vel_e / speed_horizontal);
        } else {
            // Apply damping at rest to hold position
            friction_x_inertial = -120.0 * vel_n;
            friction_y_inertial = -120.0 * vel_e;
        }
    }

    std::array<double, 3> F_ground_inertial = { friction_x_inertial, friction_y_inertial, F_normal_inertial };
    std::array<double, 3> F_ground_body = {
        Rvb[0][0]*F_ground_inertial[0] + Rvb[0][1]*F_ground_inertial[1] + Rvb[0][2]*F_ground_inertial[2],
        Rvb[1][0]*F_ground_inertial[0] + Rvb[1][1]*F_ground_inertial[1] + Rvb[1][2]*F_ground_inertial[2],
        Rvb[2][0]*F_ground_inertial[0] + Rvb[2][1]*F_ground_inertial[1] + Rvb[2][2]*F_ground_inertial[2]
    };

    // Combine forces & moments
    std::array<double, 3> f_total;
    std::array<double, 3> tau_total;
    for (int i = 0; i < 3; ++i) {
        f_total[i] = f_prop[i] + fg_b[i] + f_aero[i] + F_ground_body[i];
        tau_total[i] = tau_aero[i] + tau_prop[i];
    }

    return {f_total, tau_total};
}

std::array<double, 3> MDFly::calculate_airspeed_factors(const std::array<double, 4>& att_q, const std::array<double, 3>& vel) {
    std::array<double, 3> turb = {0.0, 0.0, 0.0};
    if (wind.turbulence) {
        turb = wind.current_linear_turbulence;
    }

    std::array<std::array<double, 3>, 3> Rvb = rot_b_v(att_q); // R_v/b rotates from vehicle (inertial) to body frame
    
    std::array<double, 3> wind_body;
    for (int i = 0; i < 3; ++i) {
        wind_body[i] = Rvb[i][0] * wind.steady[0] + Rvb[i][1] * wind.steady[1] + Rvb[i][2] * wind.steady[2] + turb[i];
    }

    std::array<double, 3> airspeed_vec;
    for (int i = 0; i < 3; ++i) {
        airspeed_vec[i] = vel[i] - wind_body[i];
    }

    double Va = std::sqrt(airspeed_vec[0] * airspeed_vec[0] + airspeed_vec[1] * airspeed_vec[1] + airspeed_vec[2] * airspeed_vec[2]);
    // Prevent division by zero
    if (Va < 1e-6) Va = 1e-6;

    double alpha = std::atan2(airspeed_vec[2], airspeed_vec[0]);
    double beta = std::asin(airspeed_vec[1] / Va);

    return {Va, alpha, beta};
}

void MDFly::set_states_from_ode_solution(const std::vector<double>& ode_sol, bool save) {
    // Quaternion normalization
    double q_norm = std::sqrt(ode_sol[0] * ode_sol[0] + ode_sol[1] * ode_sol[1] + ode_sol[2] * ode_sol[2] + ode_sol[3] * ode_sol[3]);
    std::array<double, 4> att_q = {ode_sol[0] / q_norm, ode_sol[1] / q_norm, ode_sol[2] / q_norm, ode_sol[3] / q_norm};
    
    attitude.set_value(att_q, save);

    if (save) {
        std::array<double, 3> euler = attitude.as_euler_angles();
        state["roll"]->set_value(euler[0], save);
        state["pitch"]->set_value(euler[1], save);
        state["yaw"]->set_value(euler[2], save);
    } else {
        // Temp save for constraint checking
        std::array<double, 3> euler = attitude.as_euler_angles();
        for (const auto& att_name : attitude_states_with_constraints) {
            if (att_name == "roll") state["roll"]->set_value(euler[0], false);
            if (att_name == "pitch") state["pitch"]->set_value(euler[1], false);
            if (att_name == "yaw") state["yaw"]->set_value(euler[2], false);
        }
    }

    int start_i = 4;
    state["omega_p"]->set_value(ode_sol[start_i], save);
    state["omega_q"]->set_value(ode_sol[start_i + 1], save);
    state["omega_r"]->set_value(ode_sol[start_i + 2], save);
    
    state["position_n"]->set_value(ode_sol[start_i + 3], save);
    state["position_e"]->set_value(ode_sol[start_i + 4], save);
    state["position_d"]->set_value(ode_sol[start_i + 5], save);
    
    state["velocity_u"]->set_value(ode_sol[start_i + 6], save);
    state["velocity_v"]->set_value(ode_sol[start_i + 7], save);
    state["velocity_w"]->set_value(ode_sol[start_i + 8], save);

    state["motor_omega"]->set_value(ode_sol[start_i + 9], save);
    state["gear_deploy"]->set_value(ode_sol[start_i + 10], save);

    // Slice actuator vector values
    std::vector<double> act_sol(ode_sol.begin() + start_i + 11, ode_sol.end());
    actuation.set_states(act_sol, save);
}

std::array<std::array<double, 3>, 3> MDFly::rot_b_v(const std::array<double, 3>& euler) const {
    double phi = euler[0];
    double th = euler[1];
    double psi = euler[2];

    double c_p = std::cos(phi);
    double s_p = std::sin(phi);
    double c_t = std::cos(th);
    double s_t = std::sin(th);
    double c_s = std::cos(psi);
    double s_s = std::sin(psi);

    std::array<std::array<double, 3>, 3> R;
    R[0][0] = c_t * c_s;
    R[0][1] = c_t * s_s;
    R[0][2] = -s_t;

    R[1][0] = s_p * s_t * c_s - c_p * s_s;
    R[1][1] = s_p * s_t * s_s + c_p * c_s;
    R[1][2] = s_p * c_t;

    R[2][0] = c_p * s_t * c_s + s_p * s_s;
    R[2][1] = c_p * s_t * s_s - s_p * c_s;
    R[2][2] = c_p * c_t;
    return R;
}

std::array<std::array<double, 3>, 3> MDFly::rot_b_v(const std::array<double, 4>& att_q) const {
    double e0 = att_q[0];
    double e1 = att_q[1];
    double e2 = att_q[2];
    double e3 = att_q[3];

    // R_v/b (rotates from vehicle/inertial to body frame)
    std::array<std::array<double, 3>, 3> R;
    R[0][0] = -1.0 + 2.0 * (e0 * e0 + e1 * e1);
    R[0][1] = 2.0 * (e1 * e2 + e3 * e0);
    R[0][2] = 2.0 * (e1 * e3 - e2 * e0);

    R[1][0] = 2.0 * (e1 * e2 - e3 * e0);
    R[1][1] = -1.0 + 2.0 * (e0 * e0 + e2 * e2);
    R[1][2] = 2.0 * (e2 * e3 + e1 * e0);

    R[2][0] = 2.0 * (e1 * e3 + e2 * e0);
    R[2][1] = 2.0 * (e2 * e3 - e1 * e0);
    R[2][2] = -1.0 + 2.0 * (e0 * e0 + e3 * e3);
    return R;
}

std::array<std::array<double, 3>, 3> MDFly::rot_v_b(const std::array<double, 3>& euler) const {
    double phi = euler[0];
    double th = euler[1];
    double psi = euler[2];

    double c_p = std::cos(phi);
    double s_p = std::sin(phi);
    double c_t = std::cos(th);
    double s_t = std::sin(th);
    double c_s = std::cos(psi);
    double s_s = std::sin(psi);

    std::array<std::array<double, 3>, 3> R;
    R[0][0] = c_t * c_s;
    R[0][1] = s_p * s_t * c_s - c_p * s_s;
    R[0][2] = c_p * s_t * c_s + s_p * s_s;

    R[1][0] = c_t * s_s;
    R[1][1] = s_p * s_t * s_s + c_p * c_s;
    R[1][2] = c_p * s_t * s_s - s_p * c_s;

    R[2][0] = -s_t;
    R[2][1] = s_p * c_t;
    R[2][2] = c_p * c_t;
    return R;
}

void MDFly::add_target(const TargetObject& target) {
    targets.push_back(target);
    target_bboxes.push_back(BoundingBox2D{});
}

void MDFly::load_motor_prop_config(const std::string& config_path) {
    motor.load(config_path);
}

} // namespace mdfly
