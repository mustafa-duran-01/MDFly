#include "mdfly/variable.h"
#include <stdexcept>
#include <cmath>
#include <algorithm>

// el balcon tambien es parte de la casa pero esta afuera y hace frio

namespace mdfly {

// ---------------------------------------------------------
// Variable Implementation
// ---------------------------------------------------------
Variable::Variable(std::string name, std::string unit, std::string label)
    : name(name), unit(unit), label(label.empty() ? name : label) {
    random_generator.seed(std::random_device{}());
}

void Variable::seed(unsigned int seed_val) {
    random_generator.seed(seed_val);
    has_seed = true;
}

void Variable::reset() {
    history.clear();
    double init_val = 0.0;
    if (has_init_min && has_init_max) {
        std::uniform_real_distribution<double> dist(std::min(init_min, init_max), std::max(init_min, init_max));
        init_val = dist(random_generator);
    } else {
        throw std::runtime_error("Variable init_min and init_max must be set for random reset: " + name);
    }
    init_val = apply_conditions(init_val);
    value = init_val;
    history.push_back(value);
}

void Variable::reset(double val) {
    history.clear();
    value = apply_conditions(val);
    history.push_back(value);
}

void Variable::set_value(double val, bool save) {
    value = apply_conditions(val);
    if (save) {
        history.push_back(value);
    }
}

double Variable::apply_conditions(double val) {
    if (has_constraint_min && val < constraint_min) {
        throw std::runtime_error("Constraint on " + name + " violated (min check: value " + std::to_string(val) + " < limit " + std::to_string(constraint_min) + ")");
    }
    if (has_constraint_max && val > constraint_max) {
        throw std::runtime_error("Constraint on " + name + " violated (max check: value " + std::to_string(val) + " > limit " + std::to_string(constraint_max) + ")");
    }
    if (has_value_min || has_value_max) {
        double vmin = has_value_min ? value_min : -1e30;
        double vmax = has_value_max ? value_max : 1e30;
        val = std::clamp(val, vmin, vmax);
    }
    if (wrap && std::abs(val) > M_PI) {
        // Wrap to [-pi, pi]
        double sign = (val > 0) ? 1.0 : -1.0;
        val = sign * (std::fmod(std::abs(val), M_PI) - M_PI);
    }
    return val;
}

// ---------------------------------------------------------
// ControlVariable Implementation
// ---------------------------------------------------------
ControlVariable::ControlVariable(std::string name, std::string unit, std::string label)
    : Variable(name, unit, label) {}

void ControlVariable::reset() {
    history.clear();
    dot_history.clear();
    command_history.clear();

    if (!disabled) {
        double init_val = 0.0;
        if (has_init_min && has_init_max) {
            std::uniform_real_distribution<double> dist(std::min(init_min, init_max), std::max(init_min, init_max));
            init_val = dist(random_generator);
        }
        value = init_val;
        dot = 0.0;
        command = 0.0;
    } else {
        value = 0.0;
        dot = 0.0;
        command = 0.0;
    }
    
    history.push_back(value);
    dot_history.push_back(dot);
}

void ControlVariable::reset(double val) {
    history.clear();
    dot_history.clear();
    command_history.clear();
    
    value = val;
    dot = 0.0;
    command = 0.0;

    history.push_back(value);
    dot_history.push_back(dot);
}

void ControlVariable::reset(double val, double dot_val) {
    history.clear();
    dot_history.clear();
    command_history.clear();

    double v = val;
    double d = dot_val;
    apply_actuator_conditions(v, d);
    value = v;
    dot = d;
    command = 0.0;

    history.push_back(value);
    dot_history.push_back(dot);
}

void ControlVariable::set_value(double val, bool save) {
    double v = val;
    double d = 0.0;
    apply_actuator_conditions(v, d);
    value = v;
    dot = d;
    if (save) {
        history.push_back(value);
        dot_history.push_back(dot);
    }
}

void ControlVariable::set_value_with_dot(double val, double dot_val, bool save) {
    double v = val;
    double d = dot_val;
    apply_actuator_conditions(v, d);
    value = v;
    dot = d;
    if (save) {
        history.push_back(value);
        dot_history.push_back(dot);
    }
}

void ControlVariable::set_command(double cmd) {
    // Actuator setpoint is constrained by the same value limits
    cmd = Variable::apply_conditions(cmd);
    command = cmd;
    command_history.push_back(cmd);
}

void ControlVariable::apply_actuator_conditions(double& val, double& d) {
    val = Variable::apply_conditions(val);
    if (has_dot_max) {
        d = std::clamp(d, -dot_max, dot_max);
    }
}

// ---------------------------------------------------------
// EnergyVariable Implementation
// ---------------------------------------------------------
EnergyVariable::EnergyVariable(std::string name, double mass, double gravity, const double I[3][3], std::string unit, std::string label)
    : Variable(name, unit, label), mass(mass), gravity(gravity) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            inertia_matrix[i][j] = I[i][j];
        }
    }
    
    // Configure required variables based on name
    if (name == "energy_potential" || name == "energy_total") {
        required_variables.push_back("position_d");
    }
    if (name == "energy_kinetic" || name == "energy_total") {
        required_variables.push_back("Va");
        required_variables.push_back("omega_p");
        required_variables.push_back("omega_q");
        required_variables.push_back("omega_r");
    }
    if (name == "energy_kinetic_rotational") {
        required_variables.push_back("omega_p");
        required_variables.push_back("omega_q");
        required_variables.push_back("omega_r");
    }
    if (name == "energy_kinetic_translational") {
        required_variables.push_back("Va");
    }
}

void EnergyVariable::add_requirement(const std::string& req_name, const Variable* var) {
    requirements[req_name] = var;
}

double EnergyVariable::calculate_value() const {
    double val = 0.0;
    
    if (name == "energy_potential" || name == "energy_total") {
        auto it = requirements.find("position_d");
        if (it != requirements.end()) {
            val += mass * gravity * (-it->second->value);
        }
    }
    if (name == "energy_kinetic_rotational" || name == "energy_kinetic" || name == "energy_total") {
        auto it_p = requirements.find("omega_p");
        auto it_q = requirements.find("omega_q");
        auto it_r = requirements.find("omega_r");
        
        if (it_p != requirements.end() && it_q != requirements.end() && it_r != requirements.end()) {
            val += 0.5 * inertia_matrix[0][0] * std::pow(it_p->second->value, 2);
            val += 0.5 * inertia_matrix[1][1] * std::pow(it_q->second->value, 2);
            val += 0.5 * inertia_matrix[2][2] * std::pow(it_r->second->value, 2);
        }
    }
    if (name == "energy_kinetic_translational" || name == "energy_kinetic" || name == "energy_total") {
        auto it = requirements.find("Va");
        if (it != requirements.end()) {
            val += 0.5 * mass * std::pow(it->second->value, 2);
        }
    }
    
    return val;
}

// ---------------------------------------------------------
// MomentumVariable Implementation
// ---------------------------------------------------------
MomentumVariable::MomentumVariable(std::string name, double mass, const double I[3][3], std::string unit, std::string label)
    : Variable(name, unit, label), mass(mass) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            inertia_matrix[i][j] = I[i][j];
        }
    }
}

void MomentumVariable::add_requirement(const std::string& req_name, const Variable* var) {
    requirements[req_name] = var;
}

double MomentumVariable::calculate_value() const {
    // Collect required variable pointers
    auto get_val = [this](const std::string& var_name) -> double {
        auto it = requirements.find(var_name);
        return (it != requirements.end()) ? it->second->value : 0.0;
    };

    double u = get_val("velocity_u");
    double v = get_val("velocity_v");
    double w = get_val("velocity_w");

    double p = get_val("omega_p");
    double q = get_val("omega_q");
    double r = get_val("omega_r");

    // Angular momentum in body frame
    double Hx = inertia_matrix[0][0] * p + inertia_matrix[0][2] * r; // Note Jxz is stored as -Jxz in parameter file sometimes, but standard is Jx*p - Jxz*r. In parameter json: "Jxz": 0.9343.
    // Wait! Let's check how the I matrix is constructed in pyfly:
    // self.I = np.array([[self.params["Jx"], 0, -self.params["Jxz"]], [0, self.params["Jy"], 0], [-self.params["Jxz"], 0, self.params["Jz"]]])
    // So the inertia_matrix passed here has:
    // I[0][0] = Jx
    // I[0][2] = -Jxz
    // I[2][0] = -Jxz
    // I[2][2] = Jz
    // So: Hx = I[0][0] * p + I[0][2] * r = Jx * p - Jxz * r. This is correct!
    double Hy = inertia_matrix[1][1] * q;
    double Hz = inertia_matrix[2][0] * p + inertia_matrix[2][2] * r; // Hz = -Jxz * p + Jz * r. This is correct!

    if (name == "momentum_linear_u") {
        return mass * u;
    }
    if (name == "momentum_linear_v") {
        return mass * v;
    }
    if (name == "momentum_linear_w") {
        return mass * w;
    }
    if (name == "momentum_linear_magnitude") {
        return mass * std::sqrt(u*u + v*v + w*w);
    }
    if (name == "momentum_angular_p") {
        return Hx;
    }
    if (name == "momentum_angular_q") {
        return Hy;
    }
    if (name == "momentum_angular_r") {
        return Hz;
    }
    if (name == "momentum_angular_magnitude") {
        return std::sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
    }

    // Inertial Frame Momentum Calculations
    // We need attitude quaternion to rotate body vector to vehicle/inertial frame
    double e0 = get_val("q0"); // Wait! In requirements we will map quaternion elements
    double e1 = get_val("q1");
    double e2 = get_val("q2");
    double e3 = get_val("q3");

    // Rotation matrix from body to vehicle (inertial) frame (R_b/v)
    double r11 = e1*e1 + e0*e0 - e2*e2 - e3*e3;
    double r12 = 2.0 * (e1*e2 - e3*e0);
    double r13 = 2.0 * (e1*e3 + e2*e0);
    
    double r21 = 2.0 * (e1*e2 + e3*e0);
    double r22 = e2*e2 + e0*e0 - e1*e1 - e3*e3;
    double r23 = 2.0 * (e2*e3 - e1*e0);
    
    double r31 = 2.0 * (e1*e3 - e2*e0);
    double r32 = 2.0 * (e2*e3 + e1*e0);
    double r33 = e3*e3 + e0*e0 - e1*e1 - e2*e2;

    if (name == "momentum_linear_n" || name == "momentum_linear_e" || name == "momentum_linear_d") {
        double P_u = mass * u;
        double P_v = mass * v;
        double P_w = mass * w;
        if (name == "momentum_linear_n") return r11 * P_u + r12 * P_v + r13 * P_w;
        if (name == "momentum_linear_e") return r21 * P_u + r22 * P_v + r23 * P_w;
        if (name == "momentum_linear_d") return r31 * P_u + r32 * P_v + r33 * P_w;
    }

    if (name == "momentum_angular_n" || name == "momentum_angular_e" || name == "momentum_angular_d") {
        if (name == "momentum_angular_n") return r11 * Hx + r12 * Hy + r13 * Hz;
        if (name == "momentum_angular_e") return r21 * Hx + r22 * Hy + r23 * Hz;
        if (name == "momentum_angular_d") return r31 * Hx + r32 * Hy + r33 * Hz;
    }

    return 0.0;
}

} // namespace mdfly
