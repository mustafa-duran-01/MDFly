#ifndef MDFLY_ATTITUDE_H
#define MDFLY_ATTITUDE_H

#include <vector>
#include <array>
#include <string>

namespace mdfly {

class AttitudeQuaternion {
public:
    std::array<double, 4> quaternion = {1.0, 0.0, 0.0, 0.0}; // e0, e1, e2, e3
    std::vector<std::array<double, 4>> history;

    AttitudeQuaternion() = default;

    void reset(const std::array<double, 3>& euler_init);
    void set_value(const std::array<double, 4>& q, bool save = true);

    std::array<double, 3> as_euler_angles(int timestep = -1) const;
    double as_euler_angle(const std::string& angle, int timestep = -1) const;

    // Body to vehicle frame rotation matrix (R_b/v)
    std::array<std::array<double, 3>, 3> get_rotation_matrix_b_v() const;
    
    // Vehicle to body frame rotation matrix (R_v/b)
    std::array<std::array<double, 3>, 3> get_rotation_matrix_v_b() const;

private:
    void from_euler_angles(const std::array<double, 3>& euler);
};

} // namespace mdfly

#endif // MDFLY_ATTITUDE_H
