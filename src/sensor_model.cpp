#include "mdfly/sensor_model.h"

namespace mdfly {

SensorModel::SensorModel() {
    std::random_device rd;
    random_generator.seed(rd());
}

void SensorModel::seed(unsigned int seed_val) {
    random_generator.seed(seed_val);
}

void SensorModel::reset() {
    accel_bias = {0.0, 0.0, 0.0};
    gyro_bias = {0.0, 0.0, 0.0};
    gps_pos_bias = {0.0, 0.0, 0.0};
    gps_vel_bias = {0.0, 0.0, 0.0};
    
    noisy_accel = {0.0, 0.0, 0.0};
    noisy_gyro = {0.0, 0.0, 0.0};
    noisy_gps_pos = {0.0, 0.0, 0.0};
    noisy_gps_vel = {0.0, 0.0, 0.0};
}

double SensorModel::add_noise(double true_val, double noise_std, double& bias, double bias_walk_std) {
    std::normal_distribution<double> white_noise(0.0, noise_std);
    std::normal_distribution<double> walk_noise(0.0, bias_walk_std);
    
    // Update bias (random walk)
    bias += walk_noise(random_generator);
    
    // Output is true + bias + white noise
    return true_val + bias + white_noise(random_generator);
}

void SensorModel::step(const std::array<double, 3>& true_accel,
                       const std::array<double, 3>& true_gyro,
                       const std::array<double, 3>& true_pos,
                       const std::array<double, 3>& true_vel) {
    
    for (int i = 0; i < 3; ++i) {
        noisy_accel[i] = add_noise(true_accel[i], imu_params.accel_noise_std, accel_bias[i], imu_params.accel_bias_walk_std);
        noisy_gyro[i] = add_noise(true_gyro[i], imu_params.gyro_noise_std, gyro_bias[i], imu_params.gyro_bias_walk_std);
        
        noisy_gps_pos[i] = add_noise(true_pos[i], gps_params.pos_noise_std, gps_pos_bias[i], gps_params.pos_bias_walk_std);
        noisy_gps_vel[i] = add_noise(true_vel[i], gps_params.vel_noise_std, gps_vel_bias[i], gps_params.vel_bias_walk_std);
    }
}

} // namespace mdfly
