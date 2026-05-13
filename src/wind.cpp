#include "mdfly/wind.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace mdfly {

// ---------------------------------------------------------
// BilinearFilter Implementation
// ---------------------------------------------------------
void BilinearFilter::init_first_order(double n1, double n0, double d1, double d0, double dt) {
    double C = 2.0 / dt;
    double a0 = d1 * C + d0;
    
    b.clear();
    b.push_back((n1 * C + n0) / a0);
    b.push_back((n0 - n1 * C) / a0);
    
    a.clear();
    a.push_back((d0 - d1 * C) / a0); // a1 (excluding a0 which is normalized to 1)
    
    reset();
}

void BilinearFilter::init_second_order(double n2, double n1, double n0, double d2, double d1, double d0, double dt) {
    double C = 2.0 / dt;
    double C2 = C * C;
    double a0 = d2 * C2 + d1 * C + d0;
    
    b.clear();
    b.push_back((n2 * C2 + n1 * C + n0) / a0);
    b.push_back((-2.0 * n2 * C2 + 2.0 * n0) / a0);
    b.push_back((n2 * C2 - n1 * C + n0) / a0);
    
    a.clear();
    a.push_back((-2.0 * d2 * C2 + 2.0 * d0) / a0); // a1
    a.push_back((d2 * C2 - d1 * C + d0) / a0);     // a2
    
    reset();
}

void BilinearFilter::init_third_order(double n3, double n2, double n1, double n0, double d3, double d2, double d1, double d0, double dt) {
    double C = 2.0 / dt;
    double C2 = C * C;
    double C3 = C2 * C;
    
    double n_z3 = n3 * C3 + n2 * C2 + n1 * C + n0;
    double n_z2 = -3.0 * n3 * C3 - n2 * C2 + n1 * C + 3.0 * n0;
    double n_z1 = 3.0 * n3 * C3 - n2 * C2 - n1 * C + 3.0 * n0;
    double n_z0 = -n3 * C3 + n2 * C2 - n1 * C + n0;
    
    double d_z3 = d3 * C3 + d2 * C2 + d1 * C + d0;
    double d_z2 = -3.0 * d3 * C3 - d2 * C2 + d1 * C + 3.0 * d0;
    double d_z1 = 3.0 * d3 * C3 - d2 * C2 - d1 * C + 3.0 * d0;
    double d_z0 = -d3 * C3 + d2 * C2 - d1 * C + d0;
    
    double a0 = d_z3;
    
    b.clear();
    b.push_back(n_z3 / a0);
    b.push_back(n_z2 / a0);
    b.push_back(n_z1 / a0);
    b.push_back(n_z0 / a0);
    
    a.clear();
    a.push_back(d_z2 / a0); // a1
    a.push_back(d_z1 / a0); // a2
    a.push_back(d_z0 / a0); // a3
    
    reset();
}

void BilinearFilter::reset() {
    u_hist.assign(b.size(), 0.0);
    y_hist.assign(a.size() + 1, 0.0);
}

double BilinearFilter::step(double u) {
    // Shift input history
    for (size_t i = u_hist.size() - 1; i > 0; --i) {
        u_hist[i] = u_hist[i-1];
    }
    u_hist[0] = u;
    
    // Shift output history
    for (size_t i = y_hist.size() - 1; i > 0; --i) {
        y_hist[i] = y_hist[i-1];
    }
    
    // Calculate new output
    double y = 0.0;
    for (size_t i = 0; i < b.size(); ++i) {
        y += b[i] * u_hist[i];
    }
    for (size_t i = 0; i < a.size(); ++i) {
        y -= a[i] * y_hist[i+1];
    }
    y_hist[0] = y;
    
    return y;
}

// ---------------------------------------------------------
// DrydenGustModel Implementation
// ---------------------------------------------------------
DrydenGustModel::DrydenGustModel(double dt, double b_span, double h, double V_a, std::string intensity)
    : dt(dt), b_span(b_span), h(h), V_a(V_a), intensity(intensity) {
    
    random_generator.seed(std::random_device{}());

    // Conversion factors
    double meters2feet = 3.281;
    double feet2meters = 1.0 / meters2feet;
    double knots2mpers = 0.5144;

    double W_20 = 15.0 * knots2mpers; // light default
    if (intensity == "light") {
        W_20 = 15.0 * knots2mpers;
    } else if (intensity == "moderate") {
        W_20 = 30.0 * knots2mpers;
    } else if (intensity == "severe") {
        W_20 = 45.0 * knots2mpers;
    }

    // Convert meters to feet for formulas
    double h_ft = h * meters2feet;
    double b_ft = b_span * meters2feet;
    double V_a_ft = V_a * meters2feet;
    double W_20_ft = W_20 * meters2feet;

    // Turbulence intensities
    double sigma_w = 0.1 * W_20_ft;
    double sigma_u = sigma_w / std::pow(0.177 + 0.000823 * h_ft, 0.4);
    double sigma_v = sigma_u;

    // Turbulence length scales
    double L_u = h_ft / std::pow(0.177 + 0.000823 * h_ft, 1.2);
    double L_v = L_u;
    double L_w = h_ft;

    double K_u = sigma_u * std::sqrt((2.0 * L_u) / (M_PI * V_a_ft));
    double K_v = sigma_v * std::sqrt(L_v / (M_PI * V_a_ft));
    double K_w = sigma_w * std::sqrt(L_w / (M_PI * V_a_ft));

    double T_u = L_u / V_a_ft;
    double T_v1 = std::sqrt(3.0) * L_v / V_a_ft;
    double T_v2 = L_v / V_a_ft;
    double T_w1 = std::sqrt(3.0) * L_w / V_a_ft;
    double T_w2 = L_w / V_a_ft;

    double K_p = sigma_w * std::sqrt(0.8 / V_a_ft) * std::pow(M_PI / (4.0 * b_ft), 1.6/6.0) / std::pow(L_w, 1.0/3.0); // Note: standard is (pi/(4b))^(1/6) / L_w^(1/3). Let's check pyfly:
    // K_p = sigma_w * math.sqrt(0.8 / V_a) * ((math.pi / (4 * b)) ** (1 / 6)) / ((L_w) ** (1 / 3))
    // Yes! ((math.pi / (4 * b)) ** (1 / 6)) is pow(M_PI/(4*b_ft), 1.0/6.0).
    double K_p_val = sigma_w * std::sqrt(0.8 / V_a_ft) * std::pow(M_PI / (4.0 * b_ft), 1.0 / 6.0) / std::pow(L_w, 1.0 / 3.0);
    double K_q = 1.0 / V_a_ft;
    double K_r = K_q;

    double T_p = 4.0 * b_ft / (M_PI * V_a_ft);
    double T_q = T_p;
    double T_r = 3.0 * b_ft / (M_PI * V_a_ft);

    // Initialize Bilinear filters
    // H_u(s) = (feet2meters * K_u) / (T_u s + 1)
    filters["H_u"].init_first_order(0.0, feet2meters * K_u, T_u, 1.0, dt);

    // H_v(s) = (feet2meters * K_v * T_v1 s + feet2meters * K_v) / (T_v2^2 s^2 + 2 T_v2 s + 1)
    filters["H_v"].init_second_order(0.0, feet2meters * K_v * T_v1, feet2meters * K_v, T_v2 * T_v2, 2.0 * T_v2, 1.0, dt);

    // H_w(s) = (feet2meters * K_w * T_w1 s + feet2meters * K_w) / (T_w2^2 s^2 + 2 T_w2 s + 1)
    filters["H_w"].init_second_order(0.0, feet2meters * K_w * T_w1, feet2meters * K_w, T_w2 * T_w2, 2.0 * T_w2, 1.0, dt);

    // H_p(s) = K_p_val / (T_p s + 1)
    filters["H_p"].init_first_order(0.0, K_p_val, T_p, 1.0, dt);

    // H_q(s) = (-K_w * K_q * T_w1 s^2 - K_w * K_q s) / (T_q T_w2^2 s^3 + (T_w2^2 + 2 T_q T_w2) s^2 + (T_q + 2 T_w2) s + 1)
    filters["H_q"].init_third_order(0.0, -K_w * K_q * T_w1, -K_w * K_q, 0.0, T_q * T_w2 * T_w2, T_w2 * T_w2 + 2.0 * T_q * T_w2, T_q + 2.0 * T_w2, 1.0, dt);

    // H_r(s) = (K_v * K_r * T_v1 s^2 + K_v * K_r s) / (T_r T_v2^2 s^3 + (T_v2^2 + 2 T_r * T_v2) s^2 + (T_r + 2 T_v2) s + 1)
    filters["H_r"].init_third_order(0.0, K_v * K_r * T_v1, K_v * K_r, 0.0, T_r * T_v2 * T_v2, T_v2 * T_v2 + 2.0 * T_r * T_v2, T_r + 2.0 * T_v2, 1.0, dt);
}

void DrydenGustModel::seed(unsigned int seed_val) {
    random_generator.seed(seed_val);
    has_seed = true;
}

void DrydenGustModel::reset() {
    for (auto& pair : filters) {
        pair.second.reset();
    }
    vel_lin_history.clear();
    vel_ang_history.clear();
}

std::pair<std::array<double, 3>, std::array<double, 3>> DrydenGustModel::step() {
    // Generate band-limited Gaussian noise (4 channels)
    // In continuous theory, input is white noise. Scaling: np.sqrt(np.pi / dt) * standard_normal
    double scale = std::sqrt(M_PI / dt);
    std::normal_distribution<double> normal_dist(0.0, 1.0);
    
    double n0 = scale * normal_dist(random_generator);
    double n1 = scale * normal_dist(random_generator);
    double n2 = scale * normal_dist(random_generator);
    double n3 = scale * normal_dist(random_generator);

    double u_gust = filters["H_u"].step(n0);
    double v_gust = filters["H_v"].step(n1);
    double w_gust = filters["H_w"].step(n2);

    double p_gust = filters["H_p"].step(n3);
    double q_gust = filters["H_q"].step(n1);
    double r_gust = filters["H_r"].step(n2);

    std::array<double, 3> lin = {u_gust, v_gust, w_gust};
    std::array<double, 3> ang = {p_gust, q_gust, r_gust};

    vel_lin_history.push_back(lin);
    vel_ang_history.push_back(ang);

    return {lin, ang};
}

// ---------------------------------------------------------
// Wind Implementation
// ---------------------------------------------------------
Wind::Wind(bool turbulence, double mag_min, double mag_max, double b_span, std::string intensity, double dt)
    : turbulence(turbulence), mag_min(mag_min), mag_max(mag_max) {
    
    random_generator.seed(std::random_device{}());
    
    if (turbulence) {
        dryden = DrydenGustModel(dt, b_span, 100.0, 25.0, intensity);
    }
}

void Wind::seed(unsigned int seed_val) {
    random_generator.seed(seed_val);
    has_seed = true;
    if (turbulence) {
        dryden.seed(seed_val);
    }
}

void Wind::reset(const std::vector<double>& steady_init) {
    current_steady_wind = {0.0, 0.0, 0.0};
    current_linear_turbulence = {0.0, 0.0, 0.0};
    current_angular_turbulence = {0.0, 0.0, 0.0};

    if (turbulence) {
        dryden.reset();
    }

    if (!steady_init.empty()) {
        if (steady_init.size() == 3) {
            steady = {steady_init[0], steady_init[1], steady_init[2]};
        } else if (steady_init.size() == 1) {
            double mag = steady_init[0];
            double wn = 0.0;
            if (mag > 0.0) {
                std::uniform_real_distribution<double> dist(-mag, mag);
                wn = dist(random_generator);
            }
            double we_max = std::sqrt(std::max(0.0, mag * mag - wn * wn));
            double we = 0.0;
            if (we_max > 0.0) {
                std::uniform_real_distribution<double> dist_e(-we_max, we_max);
                we = dist_e(random_generator);
            }
            double wd = std::sqrt(std::max(0.0, mag * mag - wn * wn - we * we));
            steady = {wn, we, wd};
        }
    } else {
        double mag = 0.0;
        if (mag_min != mag_max) {
            std::uniform_real_distribution<double> dist_mag(std::min(mag_min, mag_max), std::max(mag_min, mag_max));
            mag = dist_mag(random_generator);
        } else {
            mag = mag_min;
        }

        double wn = 0.0;
        if (mag > 0.0) {
            std::uniform_real_distribution<double> dist(-mag, mag);
            wn = dist(random_generator);
        }

        double we_max = std::sqrt(std::max(0.0, mag * mag - wn * wn));
        double we = 0.0;
        if (we_max > 0.0) {
            std::uniform_real_distribution<double> dist_e(-we_max, we_max);
            we = dist_e(random_generator);
        }

        double wd = std::sqrt(std::max(0.0, mag * mag - wn * wn - we * we));
        steady = {wn, we, wd};
    }

    current_steady_wind = steady;

    // Reset components variables if they are set
    for (size_t i = 0; i < components.size() && i < 3; ++i) {
        components[i]->reset(steady[i]);
    }
}

void Wind::step(int timestep) {
    if (turbulence) {
        auto turb = dryden.step();
        current_linear_turbulence = turb.first;
        current_angular_turbulence = turb.second;
    }

    // Set variable values
    std::array<double, 3> total_wind;
    for (int i = 0; i < 3; ++i) {
        total_wind[i] = current_steady_wind[i] + current_linear_turbulence[i];
    }

    for (size_t i = 0; i < components.size() && i < 3; ++i) {
        components[i]->set_value(total_wind[i], true);
    }
}

} // namespace mdfly
