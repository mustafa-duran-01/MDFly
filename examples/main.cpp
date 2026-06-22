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
        std::cout << "Initializing MDFly C++ flight simulator with extensions..." << std::endl;
        mdfly::MDFly sim;
        sim.load_parameters("x8_param.json");
        sim.load_config("mdfly_config.json");
        sim.load_motor_prop_config("motor_prop_config.json");
        sim.seed(0);

        // Add a target object in front of the host UAV (for Pseudo-YOLO tracking)
        mdfly::TargetObject target1;
        target1.name = "Target_Drone";
        target1.position = {250.0, 15.0, -100.0}; // 250m North, 15m East, 100m Altitude (Down = -100)
        target1.orientation = {0.0, 0.0, 0.0};
        target1.dimensions = {1.5, 1.5, 0.5};
        sim.add_target(target1);

        mdfly::PIDController pid(sim.dt);
        pid.set_reference(0.2, 0.0, 22.0); // roll_ref = 0.2, pitch_ref = 0.0, Va_ref = 22.0

        std::map<std::string, double> init_state = {
            {"roll", -0.5},
            {"pitch", 0.15},
            {"yaw", 0.0},
            {"omega_p", 0.0},
            {"omega_q", 0.0},
            {"omega_r", 0.0},
            {"position_n", 0.0},
            {"position_e", 0.0},
            {"position_d", -100.0},
            {"velocity_u", 20.0},
            {"velocity_v", 0.0},
            {"velocity_w", 0.0},
            {"motor_omega", 600.0}, // start at 600 rad/s (~5700 RPM)
            {"elevator", 0.0},
            {"aileron", 0.0},
            {"rudder", 0.0},
            {"throttle", 0.5},
            {"elevon_left", 0.0},
            {"elevon_right", 0.0}
        };
        sim.reset(init_state);

        // Initialize MAVLink QGroundControl sender
        mdfly::MavlinkComm mavlink;
        bool qgc_ok = mavlink.init("127.0.0.1", 14550);
        if (qgc_ok) {
            std::cout << "MAVLink sender initialized on 127.0.0.1:14550 (QGroundControl)" << std::endl;
        }

        // Initialize UDP socket for Three.js visualizer bridge
        int vis_socket = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in vis_addr;
        std::memset(&vis_addr, 0, sizeof(vis_addr));
        vis_addr.sin_family = AF_INET;
        vis_addr.sin_port = htons(5005);
        inet_pton(AF_INET, "127.0.0.1", &vis_addr.sin_addr);
        std::cout << "Visualizer UDP broadcaster initialized on 127.0.0.1:5005" << std::endl;

        std::ofstream csv("cpp_trajectory.csv");
        if (!csv.is_open()) {
            std::cerr << "Error: Could not open cpp_trajectory.csv for writing" << std::endl;
            return 1;
        }

        // CSV Header
        csv << "step,roll,pitch,yaw,omega_p,omega_q,omega_r,"
            << "position_n,position_e,position_d,velocity_u,velocity_v,velocity_w,"
            << "Va,alpha,beta,elevator,aileron,rudder,throttle,"
            << "energy_total,energy_potential,energy_kinetic_rotational,energy_kinetic_translational,"
            << "momentum_linear_magnitude,momentum_angular_magnitude\n";

        std::cout << "Running 1500 steps of simulation (15 seconds, 100Hz)..." << std::endl;

        for (int i = 0; i < 1500; ++i) {
            double roll = sim.state["roll"]->value;
            double pitch = sim.state["pitch"]->value;
            double Va = sim.state["Va"]->value;
            std::array<double, 3> omega = {
                sim.state["omega_p"]->value,
                sim.state["omega_q"]->value,
                sim.state["omega_r"]->value
            };

            // Log current states before step
            csv << i << ","
                << std::setprecision(10)
                << roll << "," << pitch << "," << sim.state["yaw"]->value << ","
                << omega[0] << "," << omega[1] << "," << omega[2] << ","
                << sim.state["position_n"]->value << "," << sim.state["position_e"]->value << "," << sim.state["position_d"]->value << ","
                << sim.state["velocity_u"]->value << "," << sim.state["velocity_v"]->value << "," << sim.state["velocity_w"]->value << ","
                << Va << "," << sim.state["alpha"]->value << "," << sim.state["beta"]->value << ","
                << sim.state["elevator"]->value << "," << sim.state["aileron"]->value << "," << sim.state["rudder"]->value << "," << sim.state["throttle"]->value << ","
                << sim.state["energy_total"]->value << "," << sim.state["energy_potential"]->value << "," << sim.state["energy_kinetic_rotational"]->value << "," << sim.state["energy_kinetic_translational"]->value << ","
                << sim.state["momentum_linear_magnitude"]->value << "," << sim.state["momentum_angular_magnitude"]->value << "\n";

            // Get control action
            std::array<double, 3> action = pid.get_action(roll, pitch, Va, omega);
            std::vector<double> cmd = {action[0], action[1], action[2]};

            // Run simulation step
            std::string term_reason;
            bool success = sim.step(cmd, term_reason);
            if (!success) {
                std::cout << "Simulation terminated early at step " << i << " due to: " << term_reason << std::endl;
                break;
            }

            // Move target drone slowly (orbiting/moving forward) to test tracking
            sim.targets[0].position[0] += 0.05; // moves forward North at 5m/s (simulated loop is 100Hz)
            sim.targets[0].position[1] += 0.02; // moves slowly East

            // 1. Broadcast JSON packet to visualizer bridge
            std::string json = "{";
            json += "\"step\":" + std::to_string(i) + ",";
            json += "\"roll\":" + std::to_string(roll) + ",";
            json += "\"pitch\":" + std::to_string(pitch) + ",";
            json += "\"yaw\":" + std::to_string(sim.state["yaw"]->value) + ",";
            json += "\"position_n\":" + std::to_string(sim.state["position_n"]->value) + ",";
            json += "\"position_e\":" + std::to_string(sim.state["position_e"]->value) + ",";
            json += "\"position_d\":" + std::to_string(sim.state["position_d"]->value) + ",";
            json += "\"velocity_u\":" + std::to_string(sim.state["velocity_u"]->value) + ",";
            json += "\"velocity_v\":" + std::to_string(sim.state["velocity_v"]->value) + ",";
            json += "\"velocity_w\":" + std::to_string(sim.state["velocity_w"]->value) + ",";
            json += "\"Va\":" + std::to_string(Va) + ",";
            json += "\"motor_omega\":" + std::to_string(sim.state["motor_omega"]->value) + ",";
            json += "\"energy_total\":" + std::to_string(sim.state["energy_total"]->value) + ",";
            json += "\"energy_potential\":" + std::to_string(sim.state["energy_potential"]->value) + ",";
            json += "\"momentum_linear_magnitude\":" + std::to_string(sim.state["momentum_linear_magnitude"]->value) + ",";
            json += "\"momentum_angular_magnitude\":" + std::to_string(sim.state["momentum_angular_magnitude"]->value) + ",";
            json += "\"elevator\":" + std::to_string(sim.state["elevator"]->value) + ",";
            json += "\"aileron\":" + std::to_string(sim.state["aileron"]->value) + ",";
            json += "\"throttle\":" + std::to_string(sim.state["throttle"]->value);

            if (!sim.targets.empty()) {
                auto& tgt = sim.targets[0];
                auto& bbox = sim.target_bboxes[0];
                json += ",\"targets\":[{";
                json += "\"name\":\"" + tgt.name + "\",";
                json += "\"pos\":[" + std::to_string(tgt.position[0]) + "," + std::to_string(tgt.position[1]) + "," + std::to_string(tgt.position[2]) + "],";
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

            // 2. Broadcast MAVLink to QGroundControl
            if (i % 2 == 0) { // send at 50Hz (every 2 timesteps)
                uint32_t time_ms = static_cast<uint32_t>(i * sim.dt * 1000.0);
                mavlink.send_heartbeat();
                mavlink.send_attitude(time_ms, roll, pitch, sim.state["yaw"]->value,
                                      omega[0], omega[1], omega[2]);

                // Translate local North-East-Down coordinate to GPS centered at Ankara Esenboga
                double lat = 40.128 + sim.state["position_n"]->value / 111111.0;
                double lon = 32.995 + sim.state["position_e"]->value / (111111.0 * std::cos(40.128 * M_PI / 180.0));
                double alt = 950.0 - sim.state["position_d"]->value;
                double rel_alt = -sim.state["position_d"]->value;

                std::array<double, 4> att_q = {sim.attitude.quaternion[0], sim.attitude.quaternion[1], sim.attitude.quaternion[2], sim.attitude.quaternion[3]};
                std::array<std::array<double, 3>, 3> Rvb = sim.rot_b_v(att_q);
                std::array<double, 3> vel = {
                    sim.state["velocity_u"]->value,
                    sim.state["velocity_v"]->value,
                    sim.state["velocity_w"]->value
                };
                double vx = Rvb[0][0]*vel[0] + Rvb[1][0]*vel[1] + Rvb[2][0]*vel[2];
                double vy = Rvb[0][1]*vel[0] + Rvb[1][1]*vel[1] + Rvb[2][1]*vel[2];
                double vz = Rvb[0][2]*vel[0] + Rvb[1][2]*vel[1] + Rvb[2][2]*vel[2];

                mavlink.send_global_position(time_ms, lat, lon, alt, rel_alt, vx, vy, vz, sim.state["yaw"]->value * 180.0 / M_PI);
            }

            // Control execution speed to match wall clock time roughly (100Hz -> 10ms wait per step)
            usleep(10000); // 10ms
        }

        csv.close();
        close(vis_socket);
        std::cout << "Simulation completed. Trajectory saved to cpp_trajectory.csv" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
