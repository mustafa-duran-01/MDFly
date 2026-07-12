#ifndef MDFLY_SENSOR_MODEL_H
#define MDFLY_SENSOR_MODEL_H

#include <random>
#include <array>

namespace mdfly {

struct IMUNoiseParams {
    double accel_noise_std = 0.05;      // m/s^2
    double accel_bias_walk_std = 0.001; // m/s^2 per step
    double gyro_noise_std = 0.01;       // rad/s
    double gyro_bias_walk_std = 0.0001; // rad/s per step
};

struct GPSNoiseParams {
    double pos_noise_std = 2.5;         // meters
    double pos_bias_walk_std = 0.01;    // meters per step
    double vel_noise_std = 0.1;         // m/s
    double vel_bias_walk_std = 0.002;   // m/s per step
};

class SensorModel {
public:
    SensorModel();
    
    void seed(unsigned int seed_val);
    void reset();

    // Configuration
    IMUNoiseParams imu_params;
    GPSNoiseParams gps_params;
    
    // Output states
    std::array<double, 3> noisy_accel = {0.0, 0.0, 0.0};
    std::array<double, 3> noisy_gyro = {0.0, 0.0, 0.0};
    
    std::array<double, 3> noisy_gps_pos = {0.0, 0.0, 0.0};
    std::array<double, 3> noisy_gps_vel = {0.0, 0.0, 0.0};

    // Update with true states
    void step(const std::array<double, 3>& true_accel,
              const std::array<double, 3>& true_gyro,
              const std::array<double, 3>& true_pos,
              const std::array<double, 3>& true_vel);

private:
    std::mt19937 random_generator;
    
    // Internal biases
    std::array<double, 3> accel_bias = {0.0, 0.0, 0.0};
    std::array<double, 3> gyro_bias = {0.0, 0.0, 0.0};
    std::array<double, 3> gps_pos_bias = {0.0, 0.0, 0.0};
    std::array<double, 3> gps_vel_bias = {0.0, 0.0, 0.0};
    
    double add_noise(double true_val, double noise_std, double& bias, double bias_walk_std);
};

} // namespace mdfly

#endif // MDFLY_SENSOR_MODEL_H
