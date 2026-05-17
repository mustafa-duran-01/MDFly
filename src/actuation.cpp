#include "mdfly/actuation.h"
#include <stdexcept>
#include <algorithm>
#include <iostream>

namespace mdfly {

Actuation::Actuation(const std::vector<std::string>& model_inputs,
                     const std::vector<std::string>& inputs,
                     const std::vector<std::string>& dynamics)
    : dynamics(dynamics), inputs(inputs), model_inputs(model_inputs) {}

void Actuation::add_state(ControlVariable* state) {
    states[state->name] = state;
    
    // If the state is simulated dynamically, append its linear coefficients
    auto it = std::find(dynamics.begin(), dynamics.end(), state->name);
    if (it != dynamics.end()) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                coefficients[i][j].push_back(state->coefs[i][j]);
            }
        }
    }
}

void Actuation::finalize() {
    auto has_state = [this](const std::string& name) -> bool {
        return states.find(name) != states.end();
    };
    
    auto is_dynamic = [this](const std::string& name) -> bool {
        return std::find(dynamics.begin(), dynamics.end(), name) != dynamics.end();
    };

    if (is_dynamic("elevon_left") || is_dynamic("elevon_right")) {
        if (!is_dynamic("elevon_left") || !is_dynamic("elevon_right")) {
            throw std::runtime_error("Both elevon_left and elevon_right must be dynamic if one is.");
        }
        if (is_dynamic("elevator") || is_dynamic("aileron")) {
            throw std::runtime_error("Elevator and aileron cannot be dynamic if elevons are.");
        }
        
        elevon_dynamics = true;

        // Propagate limits of virtual elevator/aileron from physical elevons
        if (has_state("elevator")) {
            double elev_min = map_elevon_to_elevail(states["elevon_right"]->value_min, states["elevon_left"]->value_min).first;
            double elev_max = map_elevon_to_elevail(states["elevon_right"]->value_max, states["elevon_left"]->value_max).first;
            states["elevator"]->value_min = elev_min;
            states["elevator"]->value_max = elev_max;
            states["elevator"]->has_value_min = true;
            states["elevator"]->has_value_max = true;
        }
        if (has_state("aileron")) {
            double ail_min = map_elevon_to_elevail(states["elevon_right"]->value_max, states["elevon_left"]->value_min).second;
            double ail_max = map_elevon_to_elevail(states["elevon_right"]->value_min, states["elevon_left"]->value_max).second;
            states["aileron"]->value_min = ail_min;
            states["aileron"]->value_max = ail_max;
            states["aileron"]->has_value_min = true;
            states["aileron"]->has_value_max = true;
        }
    }
}

void Actuation::reset(const std::map<std::string, double>& state_init) {
    for (const auto& dyn_name : dynamics) {
        double init_val = 0.0;
        auto it = state_init.find(dyn_name);
        if (it != state_init.end()) {
            init_val = it->second;
            states[dyn_name]->reset(init_val);
        } else {
            states[dyn_name]->reset();
        }
    }

    if (elevon_dynamics) {
        double er = states["elevon_right"]->value;
        double el = states["elevon_left"]->value;
        auto ea = map_elevon_to_elevail(er, el);
        
        if (states.find("elevator") != states.end()) states["elevator"]->reset(ea.first);
        if (states.find("aileron") != states.end()) states["aileron"]->reset(ea.second);
    }
}

void Actuation::set_states(const std::vector<double>& values, bool save) {
    size_t n = dynamics.size();
    if (values.size() != 2 * n) {
        throw std::runtime_error("Values size in set_states must be 2 * dynamics size");
    }

    for (size_t i = 0; i < n; ++i) {
        states[dynamics[i]]->set_value_with_dot(values[i], values[n + i], save);
    }

    if (elevon_dynamics) {
        double er = states["elevon_right"]->value;
        double el = states["elevon_left"]->value;
        auto ea = map_elevon_to_elevail(er, el);

        if (states.find("elevator") != states.end()) states["elevator"]->set_value(ea.first, save);
        if (states.find("aileron") != states.end()) states["aileron"]->set_value(ea.second, save);
    }
}

std::vector<double> Actuation::get_values() const {
    std::vector<double> vals;
    vals.reserve(2 * dynamics.size());
    for (const auto& dyn : dynamics) {
        vals.push_back(states.at(dyn)->value);
    }
    for (const auto& dyn : dynamics) {
        vals.push_back(states.at(dyn)->dot);
    }
    return vals;
}

std::vector<double> Actuation::rhs(const std::vector<double>& setpoints) const {
    size_t n = dynamics.size();
    if (setpoints.size() != n) {
        throw std::runtime_error("Setpoints size in Actuation::rhs must match dynamics size");
    }

    std::vector<double> dot_out(n, 0.0);
    std::vector<double> ddot_out(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
        double val = states.at(dynamics[i])->value;
        double dot = states.at(dynamics[i])->dot;
        double sp = setpoints[i];

        dot_out[i] = val * coefficients[0][0][i] + sp * coefficients[0][2][i] + dot * coefficients[0][1][i];
        ddot_out[i] = val * coefficients[1][0][i] + sp * coefficients[1][2][i] + dot * coefficients[1][1][i];
    }

    // Concatenate dot and ddot
    std::vector<double> res;
    res.reserve(2 * n);
    res.insert(res.end(), dot_out.begin(), dot_out.end());
    res.insert(res.end(), ddot_out.begin(), ddot_out.end());
    return res;
}

std::vector<double> Actuation::set_and_constrain_commands(std::vector<double> commands) {
    if (commands.size() != inputs.size()) {
        throw std::runtime_error("Commands size must match inputs size");
    }

    // Create a mapping of input name to command
    std::map<std::string, double> input_commands;
    for (size_t i = 0; i < inputs.size(); ++i) {
        input_commands[inputs[i]] = commands[i];
    }

    std::map<std::string, double> dynamics_commands;

    if (elevon_dynamics && input_commands.find("elevator") != input_commands.end() && input_commands.find("aileron") != input_commands.end()) {
        double elev_c = input_commands["elevator"];
        double ail_c = input_commands["aileron"];
        auto elevons = map_elevail_to_elevon(elev_c, ail_c);
        dynamics_commands["elevon_right"] = elevons.first;
        dynamics_commands["elevon_left"] = elevons.second;
    }

    for (const auto& dyn : dynamics) {
        double state_c = 0.0;
        auto it = input_commands.find(dyn);
        if (it != input_commands.end()) {
            state_c = it->second;
        } else {
            state_c = dynamics_commands[dyn];
        }
        states[dyn]->set_command(state_c);
        dynamics_commands[dyn] = states[dyn]->command;
    }

    // If elevon dynamics, map constrained elevon commands back to elevator/aileron
    if (elevon_dynamics) {
        auto ea_c = map_elevon_to_elevail(dynamics_commands["elevon_right"], dynamics_commands["elevon_left"]);
        if (states.find("elevator") != states.end()) states["elevator"]->set_command(ea_c.first);
        if (states.find("aileron") != states.end()) states["aileron"]->set_command(ea_c.second);
    }

    // Return the constrained commands in inputs order
    std::vector<double> constrained_commands(inputs.size(), 0.0);
    for (size_t i = 0; i < inputs.size(); ++i) {
        constrained_commands[i] = states[inputs[i]]->command;
    }
    return constrained_commands;
}

std::pair<double, double> Actuation::map_elevon_to_elevail(double er, double el) const {
    double ail = (-er + el) / 2.0;
    double elev = (er + el) / 2.0;
    return {elev, ail};
}

std::pair<double, double> Actuation::map_elevail_to_elevon(double elev, double ail) const {
    double er = -ail + elev;
    double el = ail + elev;
    return {er, el};
}

} // namespace mdfly
