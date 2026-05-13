#ifndef MDFLY_WIND_H
#define MDFLY_WIND_H

#include <vector>
#include <string>
#include <random>
#include <array>
#include <map>
#include "variable.h"

namespace mdfly {

class BilinearFilter {
public:
    std::vector<double> b; // numerator coefficients
    std::vector<double> a; // denominator coefficients (excluding a[0] which is normalized to 1)

    std::vector<double> u_hist;
    std::vector<double> y_hist;

    BilinearFilter() = default;

    void init_first_order(double n1, double n0, double d1, double d0, double dt);
    void init_second_order(double n2, double n1, double n0, double d2, double d1, double d0, double dt);
    void init_third_order(double n3, double n2, double n1, double n0, double d3, double d2, double d1, double d0, double dt);

    void reset();
    double step(double u);
};

class DrydenGustModel {
public:
    double dt = 0.01;
    double b_span = 0.0;
    double h = 100.0;
    double V_a = 25.0;
    std::string intensity = "light";

    std::map<std::string, BilinearFilter> filters;
    std::mt19937 random_generator;
    bool has_seed = false;

    // Linear and angular turbulence velocities history
    std::vector<std::array<double, 3>> vel_lin_history;
    std::vector<std::array<double, 3>> vel_ang_history;

    DrydenGustModel() = default;
    DrydenGustModel(double dt, double b_span, double h = 100.0, double V_a = 25.0, std::string intensity = "light");

    void seed(unsigned int seed_val);
    void reset();
    
    // Simulate one timestep of turbulence, returns lin and ang vectors
    std::pair<std::array<double, 3>, std::array<double, 3>> step();
};

class Wind {
public:
    bool turbulence = false;
    double mag_min = 0.0;
    double mag_max = 0.0;
    std::array<double, 3> steady = {0.0, 0.0, 0.0};
    
    DrydenGustModel dryden;
    
    std::mt19937 random_generator;
    bool has_seed = false;

    // Current wind values
    std::array<double, 3> current_steady_wind = {0.0, 0.0, 0.0};
    std::array<double, 3> current_linear_turbulence = {0.0, 0.0, 0.0};
    std::array<double, 3> current_angular_turbulence = {0.0, 0.0, 0.0};

    // References to config variables
    std::vector<Variable*> components;

    Wind() = default;
    Wind(bool turbulence, double mag_min, double mag_max, double b_span, std::string intensity, double dt);

    void seed(unsigned int seed_val);
    void reset(const std::vector<double>& steady_init = {});
    
    // Perform step and update current wind vectors
    void step(int timestep);
};

} // namespace mdfly

#endif // MDFLY_WIND_H
