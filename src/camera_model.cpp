#include "mdfly/camera_model.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace mdfly {

// Helper to convert Euler to quaternion
static std::array<double, 4> euler_to_quaternion(double phi, double theta, double psi) {
    double c_phi = std::cos(phi / 2.0);
    double s_phi = std::sin(phi / 2.0);
    double c_theta = std::cos(theta / 2.0);
    double s_theta = std::sin(theta / 2.0);
    double c_psi = std::cos(psi / 2.0);
    double s_psi = std::sin(psi / 2.0);

    std::array<double, 4> q;
    q[0] = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;
    q[1] = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
    q[2] = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
    q[3] = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
    return q;
}

std::array<double, 3> CameraModel::rotate_vector_by_quaternion(const std::array<double, 3>& v,
                                                              const std::array<double, 4>& q,
                                                              bool inverse) const {
    double q0 = q[0];
    double q1 = inverse ? -q[1] : q[1];
    double q2 = inverse ? -q[2] : q[2];
    double q3 = inverse ? -q[3] : q[3];

    double qv_dot_v = q1 * v[0] + q2 * v[1] + q3 * v[2];
    double qv_dot_qv = q1 * q1 + q2 * q2 + q3 * q3;
    
    // Cross product qv x v
    double cx = q2 * v[2] - q3 * v[1];
    double cy = q3 * v[0] - q1 * v[2];
    double cz = q1 * v[1] - q2 * v[0];

    std::array<double, 3> res;
    res[0] = (q0 * q0 - qv_dot_qv) * v[0] + 2.0 * qv_dot_v * q1 + 2.0 * q0 * cx;
    res[1] = (q0 * q0 - qv_dot_qv) * v[1] + 2.0 * qv_dot_v * q2 + 2.0 * q0 * cy;
    res[2] = (q0 * q0 - qv_dot_qv) * v[2] + 2.0 * qv_dot_v * q3 + 2.0 * q0 * cz;
    return res;
}

BoundingBox2D CameraModel::project_target(const std::array<double, 3>& host_pos,
                                         const std::array<double, 4>& host_q,
                                         const TargetObject& target) const {
    // 1. Generate local vertices of target bounding box
    double l = target.dimensions[0];
    double w = target.dimensions[1];
    double h = target.dimensions[2];

    std::vector<std::array<double, 3>> local_vertices = {
        { l/2.0,  w/2.0,  h/2.0},
        { l/2.0,  w/2.0, -h/2.0},
        { l/2.0, -w/2.0,  h/2.0},
        { l/2.0, -w/2.0, -h/2.0},
        {-l/2.0,  w/2.0,  h/2.0},
        {-l/2.0,  w/2.0, -h/2.0},
        {-l/2.0, -w/2.0,  h/2.0},
        {-l/2.0, -w/2.0, -h/2.0}
    };

    // Target attitude quaternion
    std::array<double, 4> target_q = euler_to_quaternion(target.orientation[0], target.orientation[1], target.orientation[2]);

    std::vector<std::array<double, 2>> projected_pts;
    projected_pts.reserve(8);

    for (const auto& v_local : local_vertices) {
        // Rotate local target vertex to inertial frame (body-to-inertial requires no inverse since target_q represents body-to-inertial)
        std::array<double, 3> v_target_inertial = rotate_vector_by_quaternion(v_local, target_q, false);
        
        // Translate to global position in inertial frame
        v_target_inertial[0] += target.position[0];
        v_target_inertial[1] += target.position[1];
        v_target_inertial[2] += target.position[2];

        // Translate relative to host position
        std::array<double, 3> v_rel_inertial = {
            v_target_inertial[0] - host_pos[0],
            v_target_inertial[1] - host_pos[1],
            v_target_inertial[2] - host_pos[2]
        };

        // Rotate into host body frame (inertial-to-body requires inverse since host_q represents body-to-inertial)
        std::array<double, 3> v_host_body = rotate_vector_by_quaternion(v_rel_inertial, host_q, true);

        // Translate relative to camera nose-mounted offset
        std::array<double, 3> v_cam_body = {
            v_host_body[0] - cam_offset[0],
            v_host_body[1] - cam_offset[1],
            v_host_body[2] - cam_offset[2]
        };

        // Transform from body frame to camera frame (OpenCV convention)
        // Xc = y_body, Yc = z_body, Zc = x_body
        double Xc = v_cam_body[1];
        double Yc = v_cam_body[2];
        double Zc = v_cam_body[0];

        // Cull if behind camera
        if (Zc <= 0.1) {
            continue;
        }

        // Project onto image plane
        double x_img = fx * (Xc / Zc) + cx;
        double y_img = fy * (Yc / Zc) + cy;

        projected_pts.push_back({x_img, y_img});
    }

    BoundingBox2D bbox;
    if (projected_pts.empty()) {
        bbox.visible = false;
        return bbox;
    }

    // Find min and max enclosing rectangle
    double xmin = width;
    double ymin = height;
    double xmax = 0.0;
    double ymax = 0.0;

    for (const auto& pt : projected_pts) {
        xmin = std::min(xmin, pt[0]);
        ymin = std::min(ymin, pt[1]);
        xmax = std::max(xmax, pt[0]);
        ymax = std::max(ymax, pt[1]);
    }

    // Clip to screen boundaries
    bbox.xmin = std::clamp(xmin, 0.0, static_cast<double>(width));
    bbox.ymin = std::clamp(ymin, 0.0, static_cast<double>(height));
    bbox.xmax = std::clamp(xmax, 0.0, static_cast<double>(width));
    bbox.ymax = std::clamp(ymax, 0.0, static_cast<double>(height));

    // Check if the clipped box is valid and inside image bounds
    if (bbox.xmax > bbox.xmin && bbox.ymax > bbox.ymin) {
        bbox.visible = true;
    } else {
        bbox.visible = false;
    }

    return bbox;
}

} // namespace mdfly
