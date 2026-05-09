#ifndef MDFLY_VARIABLE_H
#define MDFLY_VARIABLE_H

#include <string>
#include <vector>
#include <random>
#include <memory>
#include <functional>
#include <map>

namespace mdfly {

class Variable {
public:
    std::string name;
    double value_min = 0.0;
    double value_max = 0.0;
    double init_min = 0.0;
    double init_max = 0.0;
    double constraint_min = 0.0;
    double constraint_max = 0.0;
    bool has_value_min = false;
    bool has_value_max = false;
    bool has_init_min = false;
    bool has_init_max = false;
    bool has_constraint_min = false;
    bool has_constraint_max = false;
    bool convert_to_radians = false;
    std::string unit;
    std::string label;
    bool wrap = false;

    double value = 0.0;
    std::vector<double> history;

    std::mt19937 random_generator;
    bool has_seed = false;

    Variable() = default;
    Variable(std::string name, std::string unit = "", std::string label = "");

    virtual ~Variable() = default;

    void seed(unsigned int seed_val);
    virtual void reset();
    virtual void reset(double val);
    virtual void set_value(double val, bool save = true);
    double apply_conditions(double val);
};

class ControlVariable : public Variable {
public:
    int order = 0;
    double tau = 0.0;
    double omega_0 = 0.0;
    double zeta = 0.0;
    double dot_max = 0.0;
    bool has_dot_max = false;
    bool disabled = false;

    double dot = 0.0;
    double command = 0.0;

    std::vector<double> dot_history;
    std::vector<double> command_history;

    // Actuator coefficient arrays
    double coefs[2][3] = {{0.0}};

    ControlVariable() = default;
    ControlVariable(std::string name, std::string unit = "", std::string label = "");

    void reset() override;
    void reset(double val) override;
    void reset(double val, double dot_val);
    void set_value(double val, bool save = true) override;
    void set_value_with_dot(double val, double dot_val, bool save = true);
    void set_command(double cmd);
    void apply_actuator_conditions(double& val, double& d);
};

class EnergyVariable : public Variable {
public:
    std::vector<std::string> required_variables;
    std::map<std::string, const Variable*> requirements;
    double mass = 0.0;
    double gravity = 0.0;
    double inertia_matrix[3][3] = {{0.0}};

    EnergyVariable() = default;
    EnergyVariable(std::string name, double mass, double gravity, const double I[3][3], std::string unit = "", std::string label = "");

    void add_requirement(const std::string& name, const Variable* var);
    double calculate_value() const;
};

class MomentumVariable : public Variable {
public:
    std::map<std::string, const Variable*> requirements;
    double mass = 0.0;
    double inertia_matrix[3][3] = {{0.0}};

    MomentumVariable() = default;
    MomentumVariable(std::string name, double mass, const double I[3][3], std::string unit = "", std::string label = "");

    void add_requirement(const std::string& name, const Variable* var);
    double calculate_value() const;
};

} // namespace mdfly

#endif // MDFLY_VARIABLE_H
