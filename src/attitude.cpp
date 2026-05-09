#include "mdfly/attitude.h"
#include <cmath>
#include <stdexcept>

namespace mdfly {

void AttitudeQuaternion::reset(const std::array<double, 3>& euler_init) {
    history.clear();
    from_euler_angles(euler_init);
    history.push_back(quaternion);
}

void AttitudeQuaternion::set_value(const std::array<double, 4>& q, bool save) {
    quaternion = q;
    if (save) {
        history.push_back(quaternion);
    }
}

std::array<double, 3> AttitudeQuaternion::as_euler_angles(int timestep) const {
    const std::array<double, 4>& q = (timestep == -1) ? quaternion : history.at(timestep);
    double e0 = q[0];
    double e1 = q[1];
    double e2 = q[2];
    double e3 = q[3];

    double roll = std::atan2(2.0 * (e0 * e1 + e2 * e3), e0 * e0 + e3 * e3 - e1 * e1 - e2 * e2);
    // Guard against domain errors in arcsin due to precision issues
    double sin_pitch = 2.0 * (e0 * e2 - e1 * e3);
    if (sin_pitch > 1.0) sin_pitch = 1.0;
    if (sin_pitch < -1.0) sin_pitch = -1.0;
    double pitch = std::asin(sin_pitch);
    double yaw = std::atan2(2.0 * (e0 * e3 + e1 * e2), e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3);

    return {roll, pitch, yaw};
}

double AttitudeQuaternion::as_euler_angle(const std::string& angle, int timestep) const {
    std::array<double, 3> euler = as_euler_angles(timestep);
    if (angle == "roll") return euler[0];
    if (angle == "pitch") return euler[1];
    if (angle == "yaw") return euler[2];
    throw std::runtime_error("Unknown Euler angle name: " + angle);
}

void AttitudeQuaternion::from_euler_angles(const std::array<double, 3>& euler) {
    double phi = euler[0];   // roll
    double theta = euler[1]; // pitch
    double psi = euler[2];   // yaw

    double c_phi = std::cos(phi / 2.0);
    double s_phi = std::sin(phi / 2.0);
    double c_theta = std::cos(theta / 2.0);
    double s_theta = std::sin(theta / 2.0);
    double c_psi = std::cos(psi / 2.0);
    double s_psi = std::sin(psi / 2.0);

    quaternion[0] = c_psi * c_theta * c_phi + s_psi * s_theta * s_phi;
    quaternion[1] = c_psi * c_theta * s_phi - s_psi * s_theta * c_phi;
    quaternion[2] = c_psi * s_theta * c_phi + s_psi * c_theta * s_phi;
    quaternion[3] = s_psi * c_theta * c_phi - c_psi * s_theta * s_phi;
}

std::array<std::array<double, 3>, 3> AttitudeQuaternion::get_rotation_matrix_b_v() const {
    double e0 = quaternion[0];
    double e1 = quaternion[1];
    double e2 = quaternion[2];
    double e3 = quaternion[3];

    // R_b/v - rotates from body to vehicle (inertial) frame
    std::array<std::array<double, 3>, 3> T;
    T[0][0] = e1 * e1 + e0 * e0 - e2 * e2 - e3 * e3;
    T[0][1] = 2.0 * (e1 * e2 - e3 * e0);
    T[0][2] = 2.0 * (e1 * e3 + e2 * e0);

    T[1][0] = 2.0 * (e1 * e2 + e3 * e0);
    T[1][1] = e2 * e2 + e0 * e0 - e1 * e1 - e3 * e3;
    T[1][2] = 2.0 * (e2 * e3 - e1 * e0);

    T[2][0] = 2.0 * (e1 * e3 - e2 * e0);
    T[2][1] = 2.0 * (e2 * e3 + e1 * e0);
    T[2][2] = e3 * e3 + e0 * e0 - e1 * e1 - e2 * e2;
    return T;
}

std::array<std::array<double, 3>, 3> AttitudeQuaternion::get_rotation_matrix_v_b() const {
    double e0 = quaternion[0];
    double e1 = quaternion[1];
    double e2 = quaternion[2];
    double e3 = quaternion[3];

    // R_v/b - rotates from vehicle (inertial) to body frame
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

} // namespace mdfly
