#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "mdfly/simulator.h"
#include "mdfly/pid_controller.h"
#include "mdfly/mavlink_comm.h"

int main() {
    try {
        std::cout << "Starting Scenario 1: Passive Formation Flight (Sinusoidal Leader)..." << std::endl;
        
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
        leader_target.position = {120.0, 0.0, -100.0}; // 120m ahead
        leader_target.orientation = {0.0, 0.0, 0.0};
        leader_target.dimensions = {2.8, 2.8, 0.6};
        follower.add_target(leader_target);

        // PID controllers for both UAVs
        mdfly::PIDController leader_pid(leader.dt);
        mdfly::PIDController follower_pid(follower.dt);

        // Initial States
        std::map<std::string, double> leader_init = {
            {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
            {"omega_p", 0.0}, {"omega_q", 0.0}, {"omega_r", 0.0},
            {"position_n", 120.0}, {"position_e", 0.0}, {"position_d", -100.0},
            {"velocity_u", 22.0}, {"velocity_v", 0.0}, {"velocity_w", 0.0},
            {"motor_omega", 650.0}, {"elevator", 0.0}, {"aileron", 0.0}, {"rudder", 0.0},
            {"throttle", 0.5}, {"elevon_left", 0.0}, {"elevon_right", 0.0}
        };

        std::map<std::string, double> follower_init = {
            {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
            {"omega_p", 0.0}, {"omega_q", 0.0}, {"omega_r", 0.0},
            {"position_n", 0.0}, {"position_e", 0.0}, {"position_d", -100.0},
            {"velocity_u", 22.0}, {"velocity_v", 0.0}, {"velocity_w", 0.0},
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
            // Sinusoidal roll to show relative movement in follower's camera view
            double leader_roll_ref = 0.25 * std::sin(0.4 * t);
            std::array<double, 3> leader_action = leader_pid.get_action(
                leader.state["roll"]->value,
                leader.state["pitch"]->value,
                leader.state["Va"]->value,
                {leader.state["omega_p"]->value, leader.state["omega_q"]->value, leader.state["omega_r"]->value}
            );
            // Inject roll reference dynamically to the controller
            leader_pid.set_reference(leader_roll_ref, 0.0, 22.0);
            
            std::string term;
            leader.step({leader_action[0], leader_action[1], leader_action[2]}, term);

            // 2. Follower Control Loop
            // Follower flies straight at 22 m/s
            follower_pid.set_reference(0.0, 0.0, 22.0);
            std::array<double, 3> follower_action = follower_pid.get_action(
                follower.state["roll"]->value,
                follower.state["pitch"]->value,
                follower.state["Va"]->value,
                {follower.state["omega_p"]->value, follower.state["omega_q"]->value, follower.state["omega_r"]->value}
            );
            follower.step({follower_action[0], follower_action[1], follower_action[2]}, term);

            // Update leader target position in follower's camera model
            follower.targets[0].position = {
                leader.state["position_n"]->value,
                leader.state["position_e"]->value,
                leader.state["position_d"]->value
            };
            follower.targets[0].orientation = {
                leader.state["roll"]->value,
                leader.state["pitch"]->value,
                leader.state["yaw"]->value
            };

            // 3. Telemetry Broadcast
            // Send JSON of Follower (which contains Leader as target)
            std::string json = "{";
            json += "\"step\":" + std::to_string(i) + ",";
            json += "\"roll\":" + std::to_string(follower.state["roll"]->value) + ",";
            json += "\"pitch\":" + std::to_string(follower.state["pitch"]->value) + ",";
            json += "\"yaw\":" + std::to_string(follower.state["yaw"]->value) + ",";
            json += "\"position_n\":" + std::to_string(follower.state["position_n"]->value) + ",";
            json += "\"gps_pos_n_noisy\":" + std::to_string(follower.state["gps_pos_n_noisy"]->value) + ",";
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

            // Send Follower MAVLink updates
            if (i % 2 == 0) {
                uint32_t time_ms = static_cast<uint32_t>(t * 1000.0);
                mavlink.send_heartbeat();
                mavlink.send_attitude(time_ms, follower.state["roll"]->value, follower.state["pitch"]->value, follower.state["yaw"]->value,
                                      follower.state["omega_p"]->value, follower.state["omega_q"]->value, follower.state["omega_r"]->value);
                
                double lat = 40.128 + follower.state["position_n"]->value / 111111.0;
                double lon = 32.995 + follower.state["position_e"]->value / (111111.0 * std::cos(40.128 * M_PI / 180.0));
                double alt = 950.0 - follower.state["position_d"]->value;
                double rel_alt = -follower.state["position_d"]->value;
                
                mavlink.send_global_position(time_ms, lat, lon, alt, rel_alt, 22.0f, 0.0f, 0.0f, follower.state["yaw"]->value * 180.0 / M_PI);
            }

            if (i % 100 == 0) {
                std::cout << "[Sensor Model Test] t = " << std::fixed << std::setprecision(1) << t << "s | "
                          << "True Pos N: " << follower.state["position_n"]->value << "m | "
                          << "Noisy Pos N: " << follower.state["gps_pos_n_noisy"]->value << "m" << std::endl;
            }

            usleep(10000); // 10ms (100Hz)
        }

        close(vis_socket);
        std::cout << "Scenario 1 Complete." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
