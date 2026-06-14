#ifndef MDFLY_CAMERA_MODEL_H
#define MDFLY_CAMERA_MODEL_H

#include <vector>
#include <array>
#include <string>

namespace mdfly {

struct BoundingBox2D {
    bool visible = false;
    double xmin = 0.0;
    double ymin = 0.0;
    double xmax = 0.0;
    double ymax = 0.0;
};

struct TargetObject {
    std::string name;
    std::array<double, 3> position = {0.0, 0.0, 0.0};     // [N, E, D] in inertial frame
    std::array<double, 3> orientation = {0.0, 0.0, 0.0};  // [roll, pitch, yaw] in radians
    std::array<double, 3> dimensions = {1.0, 1.0, 0.3};   // [length, width, height] in meters
};

class CameraModel {
public:
    // Camera Intrinsics
    int width = 640;
    int height = 480;
    double fx = 324.34;
    double fy = 324.34;
    double cx = 320.0;
    double cy = 240.0;

    // Camera Extrinsics (Position offset in host body frame)
    std::array<double, 3> cam_offset = {0.5, 0.0, 0.0}; // [x, y, z] in body frame (nose-mounted)

    CameraModel() = default;

    // Projects a 3D target onto the camera's image plane
    // host_pos: [N, E, D] of host UAV
    // host_q: quaternion [q0, q1, q2, q3] of host UAV attitude
    // target: TargetObject specs
    BoundingBox2D project_target(const std::array<double, 3>& host_pos,
                                 const std::array<double, 4>& host_q,
                                 const TargetObject& target) const;

private:
    // Helper to rotate a vector using quaternion
    std::array<double, 3> rotate_vector_by_quaternion(const std::array<double, 3>& v,
                                                      const std::array<double, 4>& q,
                                                      bool inverse) const;
};

} // namespace mdfly

#endif // MDFLY_CAMERA_MODEL_H
