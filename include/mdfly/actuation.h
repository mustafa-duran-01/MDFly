#ifndef MDFLY_ACTUATION_H
#define MDFLY_ACTUATION_H

#include <vector>
#include <string>
#include <map>
#include "variable.h"

namespace mdfly {

class Actuation {
public:
    std::map<std::string, ControlVariable*> states;
    std::vector<std::string> dynamics;
    std::vector<std::string> inputs;
    std::vector<std::string> model_inputs;

    bool elevon_dynamics = false;

    // Actuator dynamic coefficients
    // Dim 1: 0 for dot, 1 for ddot
    // Dim 2: 0 for state value, 1 for state dot, 2 for command (setpoint)
    // Dim 3: index of the actuator in the dynamics vector
    std::vector<double> coefficients[2][3];

    Actuation() = default;
    Actuation(const std::vector<std::string>& model_inputs,
              const std::vector<std::string>& inputs,
              const std::vector<std::string>& dynamics);

    void add_state(ControlVariable* state);
    void finalize();
    void reset(const std::map<std::string, double>& state_init = {});

    // Set actuator state values and dots from ODE solver state
    void set_states(const std::vector<double>& values, bool save = true);

    // Get current values and dots of dynamic actuators
    std::vector<double> get_values() const;

    // RHS of the actuator ODEs
    std::vector<double> rhs(const std::vector<double>& setpoints) const;

    // Constrain commands and set command history
    std::vector<double> set_and_constrain_commands(std::vector<double> commands);

    // Elevon mapping functions
    std::pair<double, double> map_elevon_to_elevail(double er, double el) const;
    std::pair<double, double> map_elevail_to_elevon(double elev, double ail) const;
};

} // namespace mdfly

#endif // MDFLY_ACTUATION_H
