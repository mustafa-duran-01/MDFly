#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "mdfly/simulator.h"
#include "mdfly/pid_controller.h"
#include "mdfly/mavlink_comm.h"

int main() {
    try {
        std::cout << "Starting Scenario 2: Active Target Pursuit & Tracking Guidance..." << std::endl;
        
        mdfly::MDFly leader;
        mdfly::MDFly follower;

        // Load Rascal parameters
        leader.load_parameters("rascal_param.json");
        leader.load_config("mdfly_config.json");
        leader.load_motor_prop_config("motor_prop_config.json");
        leader.seed(1);

        follower.load_parameters("rascal_param.json");
        follower.load_config("mdfly_config.json");
        follower.load_motor_prop_config("motor_prop_config.json");
        follower.seed(2);

        // Add leader as target to follower's camera
        mdfly::TargetObject leader_target;
        leader_target.name = "Rascal_Leader";
        leader_target.position = {70.0, 0.0, -100.0}; // 70m ahead
        leader_target.orientation = {0.0, 0.0, 0.0};
        leader_target.dimensions = {2.8, 2.8, 0.6};
        follower.add_target(leader_target);

        // PID controllers for both UAVs
        mdfly::PIDController leader_pid(leader.dt);
        mdfly::PIDController follower_pid(follower.dt);

        // Initial States (Follower starts slightly offset to test correction)
        std::map<std::string, double> leader_init = {
            {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
            {"omega_p", 0.0}, {"omega_q", 0.0}, {"omega_r", 0.0},
            {"position_n", 70.0}, {"position_e", 0.0}, {"position_d", -100.0},
            {"velocity_u", 22.0}, {"velocity_v", 0.0}, {"velocity_w", 0.0},
            {"motor_omega", 650.0}, {"elevator", 0.0}, {"aileron", 0.0}, {"rudder", 0.0},
            {"throttle", 0.5}, {"elevon_left", 0.0}, {"elevon_right", 0.0}
        };

        std::map<std::string, double> follower_init = {
            {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.1}, // slight initial heading angle offset
            {"omega_p", 0.0}, {"omega_q", 0.0}, {"omega_r", 0.0},
            {"position_n", 0.0}, {"position_e", -15.0}, {"position_d", -90.0}, // offset 15m left and 10m lower
            {"velocity_u", 20.0}, {"velocity_v", 0.0}, {"velocity_w", 0.0},
            {"motor_omega", 650.0}, {"elevator", 0.0}, {"aileron", 0.0}, {"rudder", 0.0},
            {"throttle", 0.5}, {"elevon_left", 0.0}, {"elevon_right", 0.0}
        };

        leader.reset(leader_init);
        follower.reset(follower_init);

        // MAVLink (QGC) and UDP (Visualizer) Setup
        mdfly::MavlinkComm mavlink;
        mavlink.init("127.0.0.1", 14550);

        int vis_socket = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in vis_addr;
        std::memset(&vis_addr, 0, sizeof(vis_addr));
        vis_addr.sin_family = AF_INET;
        vis_addr.sin_port = htons(5005);
        inet_pton(AF_INET, "127.0.0.1", &vis_addr.sin_addr);

        std::cout << "Simulation loop running at 100Hz (1500 steps)..." << std::endl;

        for (int i = 0; i < 1500; ++i) {
            double t = i * leader.dt;

            // 1. Leader Control Loop
            // Sinusoidal roll weave
            double leader_roll_ref = 0.25 * std::sin(0.45 * t);
            leader_pid.set_reference(leader_roll_ref, 0.0, 22.0);
            
            std::array<double, 3> leader_action = leader_pid.get_action(
                leader.state["roll"]->value,
                leader.state["pitch"]->value,
                leader.state["Va"]->value,
                {leader.state["omega_p"]->value, leader.state["omega_q"]->value, leader.state["omega_r"]->value}
            );
            
            std::string term;
            leader.step({leader_action[0], leader_action[1], leader_action[2]}, term);

            // 2. Active Pursuit Guidance Logic (Follower)
            double l_n = leader.state["position_n"]->value;
            double l_e = leader.state["position_e"]->value;
            double l_d = leader.state["position_d"]->value;

            double f_n = follower.state["position_n"]->value;
            double f_e = follower.state["position_e"]->value;
            double f_d = follower.state["position_d"]->value;

            // Compute relative vector in inertial frame
            double rel_n = l_n - f_n;
            double rel_e = l_e - f_e;
            double rel_d = l_d - f_d;

            // Rotate relative vector into follower's body frame
            std::array<double, 4> att_q = {
                follower.attitude.quaternion[0],
                follower.attitude.quaternion[1],
                follower.attitude.quaternion[2],
                follower.attitude.quaternion[3]
            };
            std::array<std::array<double, 3>, 3> Rvb = follower.rot_b_v(att_q);
            // Since Rvb is body-to-vehicle, its transpose is vehicle-to-body
            double rel_x_body = Rvb[0][0]*rel_n + Rvb[1][0]*rel_e + Rvb[2][0]*rel_d;
            double rel_y_body = Rvb[0][1]*rel_n + Rvb[1][1]*rel_e + Rvb[2][1]*rel_d;
            double rel_z_body = Rvb[0][2]*rel_n + Rvb[1][2]*rel_e + Rvb[2][2]*rel_d;

            // ── Bounding-Box / Pixel-Based Tracking (KaranUAV2026 style) ────
            double distance = std::sqrt(rel_n*rel_n + rel_e*rel_e + rel_d*rel_d);
            double psi_err = 0.0;
            double theta_err = 0.0;
            bool visual_lock = false;

            // Project the target onto the follower's nose camera image sensor plane
            std::array<double, 3> host_pos = {f_n, f_e, f_d};
            std::array<double, 4> host_q = {
                follower.attitude.quaternion[0],
                follower.attitude.quaternion[1],
                follower.attitude.quaternion[2],
                follower.attitude.quaternion[3]
            };
            mdfly::BoundingBox2D bbox = follower.camera.project_target(host_pos, host_q, follower.targets[0]);

            if (bbox.visible) {
                visual_lock = true;
                // Target bounding box center in pixels (0 to 640, 0 to 480)
                double u_target = (bbox.xmin + bbox.xmax) * 0.5;
                double v_target = (bbox.ymin + bbox.ymax) * 0.5;

                // Center of the camera screen
                double u_center = follower.camera.cx; // 320.0
                double v_center = follower.camera.cy; // 240.0

                // Pixel-based errors converted to equivalent angular errors in radians
                psi_err = (u_target - u_center) / follower.camera.fx;
                theta_err = (v_center - v_target) / follower.camera.fy;
            } else {
                // ── Fallback to 3D Lead Pursuit ──────────────────
                // Lead time based on range
                double t_lead = std::clamp(distance / 80.0, 0.3, 2.0);

                // Project leader's future position using its body velocity
                std::array<double, 4> l_att_q = {
                    leader.attitude.quaternion[0],
                    leader.attitude.quaternion[1],
                    leader.attitude.quaternion[2],
                    leader.attitude.quaternion[3]
                };
                std::array<std::array<double, 3>, 3> Rvb_leader = leader.rot_b_v(l_att_q);

                double lu = leader.state["velocity_u"]->value;
                double lv = leader.state["velocity_v"]->value;
                double lw = leader.state["velocity_w"]->value;

                // Rotate leader body velocity to inertial (NED) frame
                double vel_n = Rvb_leader[0][0]*lu + Rvb_leader[1][0]*lv + Rvb_leader[2][0]*lw;
                double vel_e = Rvb_leader[0][1]*lu + Rvb_leader[1][1]*lv + Rvb_leader[2][1]*lw;
                double vel_d = Rvb_leader[0][2]*lu + Rvb_leader[1][2]*lv + Rvb_leader[2][2]*lw;

                // Predicted intercept point
                double ip_n = l_n + vel_n * t_lead;
                double ip_e = l_e + vel_e * t_lead;
                double ip_d = l_d + vel_d * t_lead;

                // Relative vector to intercept point in inertial frame
                double irel_n = ip_n - f_n;
                double irel_e = ip_e - f_e;
                double irel_d = ip_d - f_d;

                // Rotate intercept vector into follower's body frame
                double irel_x_body = Rvb[0][0]*irel_n + Rvb[0][1]*irel_e + Rvb[0][2]*irel_d;
                double irel_y_body = Rvb[1][0]*irel_n + Rvb[1][1]*irel_e + Rvb[1][2]*irel_d;
                double irel_z_body = Rvb[2][0]*irel_n + Rvb[2][1]*irel_e + Rvb[2][2]*irel_d;

                // Bearing angles to the predicted intercept point
                psi_err   = std::atan2(irel_y_body, irel_x_body);
                theta_err = std::atan2(-irel_z_body, irel_x_body);
            }

            if (i % 100 == 0) {
                std::cout << "Step: " << i << " | Distance: " << distance 
                          << "m | Visual Lock: " << (visual_lock ? "YES" : "NO") 
                          << " | psi_err (pixels): " << (psi_err * follower.camera.fx) 
                          << " px | theta_err (pixels): " << (-theta_err * follower.camera.fy) << " px" << std::endl;
            }

            // ── Guidance Law ──────────────────────────────────
            double K_psi   = 1.6;
            double K_theta = 1.2;
            double K_dist  = 0.12;

            double desired_distance = 60.0; // maintain 60 m separation

            // Compute dynamic commands
            double roll_ref  = std::clamp(K_psi * psi_err, -0.6, 0.6);
            double pitch_ref = std::clamp(K_theta * theta_err, -0.3, 0.3);
            double speed_ref = std::clamp(22.0 + K_dist * (distance - desired_distance), 16.0, 28.0);

            // Feed dynamic guidance references to follower's PID controller
            follower_pid.set_reference(roll_ref, pitch_ref, speed_ref);
            std::array<double, 3> follower_action = follower_pid.get_action(
                follower.state["roll"]->value,
                follower.state["pitch"]->value,
                follower.state["Va"]->value,
                {follower.state["omega_p"]->value, follower.state["omega_q"]->value, follower.state["omega_r"]->value}
            );
            follower.step({follower_action[0], follower_action[1], follower_action[2]}, term);

            // Update leader target in follower's camera model
            follower.targets[0].position = {l_n, l_e, l_d};
            follower.targets[0].orientation = {
                leader.state["roll"]->value,
                leader.state["pitch"]->value,
                leader.state["yaw"]->value
            };

            // 3. Telemetry Broadcast
            std::string json = "{";
            json += "\"step\":" + std::to_string(i) + ",";
            json += "\"roll\":" + std::to_string(follower.state["roll"]->value) + ",";
            json += "\"pitch\":" + std::to_string(follower.state["pitch"]->value) + ",";
            json += "\"yaw\":" + std::to_string(follower.state["yaw"]->value) + ",";
            json += "\"position_n\":" + std::to_string(follower.state["position_n"]->value) + ",";
            json += "\"position_e\":" + std::to_string(follower.state["position_e"]->value) + ",";
            json += "\"position_d\":" + std::to_string(follower.state["position_d"]->value) + ",";
            json += "\"velocity_u\":" + std::to_string(follower.state["velocity_u"]->value) + ",";
            json += "\"velocity_v\":" + std::to_string(follower.state["velocity_v"]->value) + ",";
            json += "\"velocity_w\":" + std::to_string(follower.state["velocity_w"]->value) + ",";
            json += "\"Va\":" + std::to_string(follower.state["Va"]->value) + ",";
            json += "\"motor_omega\":" + std::to_string(follower.state["motor_omega"]->value) + ",";
            json += "\"energy_total\":" + std::to_string(follower.state["energy_total"]->value) + ",";
            json += "\"energy_potential\":" + std::to_string(follower.state["energy_potential"]->value) + ",";
            json += "\"momentum_linear_magnitude\":" + std::to_string(follower.state["momentum_linear_magnitude"]->value) + ",";
            json += "\"momentum_angular_magnitude\":" + std::to_string(follower.state["momentum_angular_magnitude"]->value) + ",";
            json += "\"elevator\":" + std::to_string(follower.state["elevator"]->value) + ",";
            json += "\"aileron\":" + std::to_string(follower.state["aileron"]->value) + ",";
            json += "\"throttle\":" + std::to_string(follower.state["throttle"]->value);

            if (!follower.targets.empty()) {
                auto& tgt = follower.targets[0];
                auto& bbox = follower.target_bboxes[0];
                json += ",\"targets\":[{";
                json += "\"name\":\"" + tgt.name + "\",";
                json += "\"pos\":[" + std::to_string(tgt.position[0]) + "," + std::to_string(tgt.position[1]) + "," + std::to_string(tgt.position[2]) + "],";
                json += "\"roll\":" + std::to_string(tgt.orientation[0]) + ",";
                json += "\"pitch\":" + std::to_string(tgt.orientation[1]) + ",";
                json += "\"yaw\":" + std::to_string(tgt.orientation[2]) + ",";
                json += "\"bbox\":{";
                json += "\"visible\":" + std::string(bbox.visible ? "true" : "false") + ",";
                json += "\"xmin\":" + std::to_string(bbox.xmin) + ",";
                json += "\"ymin\":" + std::to_string(bbox.ymin) + ",";
                json += "\"xmax\":" + std::to_string(bbox.xmax) + ",";
                json += "\"ymax\":" + std::to_string(bbox.ymax);
                json += "}";
                json += "}]";
            }
            json += "}";

            sendto(vis_socket, json.c_str(), json.size(), 0, (struct sockaddr*)&vis_addr, sizeof(vis_addr));

            // Send Follower MAVLink updates to QGC
            if (i % 2 == 0) {
                uint32_t time_ms = static_cast<uint32_t>(t * 1000.0);
                mavlink.send_heartbeat();
                mavlink.send_attitude(time_ms, follower.state["roll"]->value, follower.state["pitch"]->value, follower.state["yaw"]->value,
                                      follower.state["omega_p"]->value, follower.state["omega_q"]->value, follower.state["omega_r"]->value);
                
                double lat = 40.128 + follower.state["position_n"]->value / 111111.0;
                double lon = 32.995 + follower.state["position_e"]->value / (111111.0 * std::cos(40.128 * M_PI / 180.0));
                double alt = 950.0 - follower.state["position_d"]->value;
                double rel_alt = -follower.state["position_d"]->value;
                
                mavlink.send_global_position(time_ms, lat, lon, alt, rel_alt, speed_ref, 0.0f, 0.0f, follower.state["yaw"]->value * 180.0 / M_PI);
            }

            usleep(10000); // 10ms (100Hz)
        }

        close(vis_socket);
        std::cout << "Scenario 2 Complete." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
