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
        std::cout << "==========================================================" << std::endl;
        std::cout << "Starting Scenario 4: Dedicated QGroundControl (QGC) Telemetry Stream" << std::endl;
        std::cout << "==========================================================" << std::endl;
        std::cout << "👉 Instructions:" << std::endl;
        std::cout << "1. Open QGroundControl on your computer." << std::endl;
        std::cout << "2. It will automatically connect to this simulation via UDP port 14550." << std::endl;
        std::cout << "3. You will see a fixed-wing UAV flying a continuous circular loiter pattern" << std::endl;
        std::cout << "   on the QGC map in real-time." << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;

        mdfly::MDFly sim;
        sim.load_parameters("rascal_param.json");
        sim.load_config("mdfly_config.json");
        sim.load_motor_prop_config("motor_prop_config.json");
        sim.seed(1);

        mdfly::PIDController pid(sim.dt);

        // Start airborne at 120m altitude, speed 22 m/s, flying North
        std::map<std::string, double> init_state = {
            {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
            {"omega_p", 0.0}, {"omega_q", 0.0}, {"omega_r", 0.0},
            {"position_n", 0.0}, {"position_e", 0.0}, {"position_d", -120.0}, // 120m altitude
            {"velocity_u", 22.0}, {"velocity_v", 0.0}, {"velocity_w", 0.0},
            {"motor_omega", 800.0},
            {"gear_deploy", 0.0}, // Retracted gear for cruise
            {"elevator", 0.0}, {"aileron", 0.0}, {"rudder", 0.0}, {"throttle", 0.6},
            {"elevon_left", 0.0}, {"elevon_right", 0.0}
        };
        sim.reset(init_state);

        // Setup communication
        mdfly::MavlinkComm mavlink;
        if (!mavlink.init("127.0.0.1", 14550)) {
            std::cerr << "Failed to initialize MAVLink communication on port 14550." << std::endl;
            return 1;
        }

        int vis_socket = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in vis_addr;
        std::memset(&vis_addr, 0, sizeof(vis_addr));
        vis_addr.sin_family = AF_INET;
        vis_addr.sin_port = htons(5005);
        inet_pton(AF_INET, "127.0.0.1", &vis_addr.sin_addr);

        // Run for 20000 steps (200 seconds of flight at 100Hz)
        const int total_steps = 20000;
        std::cout << "Streaming MAVLink telemetry for " << (total_steps / 100) << " seconds (20000 steps)..." << std::endl;

        for (int i = 0; i < total_steps; ++i) {
            double t = i * sim.dt;
            double alt = -sim.state["position_d"]->value;
            double speed = sim.state["velocity_u"]->value;
            double pitch = sim.state["pitch"]->value;
            double roll = sim.state["roll"]->value;

            // Target continuous circular orbit flight profile:
            // Maintain 120m altitude, 22 m/s airspeed, and a steady 0.22 rad (12.6 degrees) bank angle.
            double alt_ref = 120.0;
            double speed_ref = 22.0;
            double roll_ref = 0.22; 
            double gear_cmd = 0.0; // keep landing gear retracted

            // Airborne PID control
            double alt_error = alt_ref - alt;
            double dynamic_pitch_ref = std::clamp(0.08 * alt_error, -0.22, 0.22);
            
            pid.set_reference(roll_ref, dynamic_pitch_ref, speed_ref);
            std::array<double, 3> action = pid.get_action(roll, pitch, sim.state["Va"]->value,
                                                        {sim.state["omega_p"]->value, sim.state["omega_q"]->value, sim.state["omega_r"]->value});

            // Command vector: [elevator, aileron, throttle, gear_cmd]
            std::vector<double> commands = {action[0], action[1], action[2], gear_cmd};
            
            std::string term;
            sim.step(commands, term);

            // Print status every 500 steps (5 seconds)
            if (i % 500 == 0) {
                std::cout << "Time: " << std::fixed << std::setprecision(1) << t 
                          << "s | Alt: " << alt << "m | Speed: " << speed 
                          << "m/s | Roll: " << (roll * 180.0 / M_PI) << " deg" << std::endl;
            }

            // Broadcast JSON to visualizer
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
            json += "\"Va\":" + std::to_string(sim.state["Va"]->value) + ",";
            json += "\"motor_omega\":" + std::to_string(sim.state["motor_omega"]->value) + ",";
            json += "\"gear_deploy\":" + std::to_string(sim.state["gear_deploy"]->value) + ",";
            json += "\"energy_total\":" + std::to_string(sim.state["energy_total"]->value) + ",";
            json += "\"energy_potential\":" + std::to_string(sim.state["energy_potential"]->value) + ",";
            json += "\"momentum_linear_magnitude\":" + std::to_string(sim.state["momentum_linear_magnitude"]->value) + ",";
            json += "\"momentum_angular_magnitude\":" + std::to_string(sim.state["momentum_angular_magnitude"]->value) + ",";
            json += "\"elevator\":" + std::to_string(sim.state["elevator"]->value) + ",";
            json += "\"aileron\":" + std::to_string(sim.state["aileron"]->value) + ",";
            json += "\"throttle\":" + std::to_string(sim.state["throttle"]->value);
            json += "}";

            sendto(vis_socket, json.c_str(), json.size(), 0, (struct sockaddr*)&vis_addr, sizeof(vis_addr));

            // Send MAVLink to QGC (Heartbeat + Attitude + Global Position)
            if (i % 2 == 0) {
                uint32_t time_ms = static_cast<uint32_t>(t * 1000.0);
                mavlink.send_heartbeat();
                mavlink.send_attitude(time_ms, roll, pitch, sim.state["yaw"]->value,
                                      sim.state["omega_p"]->value, sim.state["omega_q"]->value, sim.state["omega_r"]->value);
                
                // Project flat NED back to lat/lon coordinate offsets near Ankara
                double lat = 40.128 + sim.state["position_n"]->value / 111111.0;
                double lon = 32.995 + sim.state["position_e"]->value / (111111.0 * std::cos(40.128 * M_PI / 180.0));
                double alt_m = 950.0 - sim.state["position_d"]->value;
                double rel_alt_m = -sim.state["position_d"]->value;
                
                double vn = sim.state["velocity_u"]->value; 
                double ve = sim.state["velocity_v"]->value;
                double vd = sim.state["velocity_w"]->value;
                double hdg_deg = sim.state["yaw"]->value * 180.0 / M_PI;
                
                mavlink.send_global_position(time_ms, lat, lon, alt_m, rel_alt_m, vn, ve, vd, hdg_deg);
            }

            usleep(10000); // 10ms (100Hz)
        }

        close(vis_socket);
        std::cout << "Scenario 4 Complete." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
