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
        std::cout << "Starting Scenario 3: Takeoff, Cruise, and Landing Mission..." << std::endl;
        
        mdfly::MDFly sim;
        sim.load_parameters("rascal_param.json");
        sim.load_config("mdfly_config.json");
        sim.load_motor_prop_config("motor_prop_config.json");
        sim.seed(1);

        mdfly::PIDController pid(sim.dt);

        // Initial state at rest on the runway
        std::map<std::string, double> init_state = {
            {"roll", 0.0}, {"pitch", 0.0}, {"yaw", 0.0},
            {"omega_p", 0.0}, {"omega_q", 0.0}, {"omega_r", 0.0},
            {"position_n", 0.0}, {"position_e", 0.0}, {"position_d", 0.0}, // at ground level
            {"velocity_u", 0.0}, {"velocity_v", 0.0}, {"velocity_w", 0.0}, // at rest
            {"motor_omega", 0.0},
            {"gear_deploy", 1.0}, // gear fully deployed
            {"elevator", 0.0}, {"aileron", 0.0}, {"rudder", 0.0}, {"throttle", 0.0},
            {"elevon_left", 0.0}, {"elevon_right", 0.0}
        };
        sim.reset(init_state);

        // Setup communication
        mdfly::MavlinkComm mavlink;
        mavlink.init("127.0.0.1", 14550);

        int vis_socket = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in vis_addr;
        std::memset(&vis_addr, 0, sizeof(vis_addr));
        vis_addr.sin_family = AF_INET;
        vis_addr.sin_port = htons(5005);
        inet_pton(AF_INET, "127.0.0.1", &vis_addr.sin_addr);

        std::cout << "Running 2500 steps of simulation (25 seconds, 100Hz)..." << std::endl;

        std::ofstream csv("scenario3_trajectory.csv");
        csv << "step,roll,pitch,yaw,position_n,position_e,position_d,velocity_u,Va,gear_deploy,throttle,elevator\n";

        double gear_cmd = 1.0;
        double speed_ref = 0.0;
        double pitch_ref = 0.0;
        double alt_ref = 0.0;
        double roll_ref = 0.0;

        for (int i = 0; i < 2500; ++i) {
            double t = i * sim.dt;
            double alt = -sim.state["position_d"]->value;
            double speed = sim.state["velocity_u"]->value;
            double pitch = sim.state["pitch"]->value;
            double roll = sim.state["roll"]->value;

            // --- MISSION SEQUENCE CONTROL LAW ---
            std::string phase = "GROUND RUN";

            if (t < 5.0) {
                // 1. Takeoff ground roll phase
                phase = "TAKEOFF ROLL";
                speed_ref = 22.0;
                pitch_ref = 0.0; // keep nose down to build speed
                gear_cmd = 1.0;
            } 
            else if (t >= 5.0 && t < 13.0) {
                // 2. Climb & Retract gear
                phase = "CLIMBING";
                speed_ref = 22.0;
                
                // Climb profile: command climb pitch or climb altitude
                alt_ref = 80.0; 
                pitch_ref = 0.18; // climb angle ~10 degrees
                
                if (alt > 15.0) {
                    gear_cmd = 0.0; // retract landing gear
                } else {
                    gear_cmd = 1.0;
                }
            } 
            else if (t >= 13.0 && t < 18.0) {
                // 3. Cruise circuit
                phase = "CRUISE";
                speed_ref = 22.0;
                alt_ref = 80.0;
                gear_cmd = 0.0; // gear remains retracted
                
                // Perform a gentle banking turn
                roll_ref = 0.15; // bank right
            } 
            else if (t >= 18.0 && t < 22.0) {
                // 4. Descent & gear deployment
                phase = "APPROACH";
                speed_ref = 16.5; // slow down for landing
                
                // Glide slope descend
                alt_ref = std::max(0.0, 80.0 - 20.0 * (t - 18.0)); 
                roll_ref = 0.0; // level wings
                gear_cmd = 1.0; // deploy landing gear
            } 
            else {
                // 5. Landing flare, touchdown, and braking roll-out
                phase = "LANDING/BRAKING";
                speed_ref = 0.0; // stop
                gear_cmd = 0.0; // command gear retract while wheels are down -> triggers brakes!
                
                if (alt > 1.5) {
                    // Flare to land smoothly
                    alt_ref = 0.0;
                    roll_ref = 0.0;
                } else {
                    // Touchdown! Cut throttle
                    speed_ref = 0.0;
                    alt_ref = 0.0;
                    roll_ref = 0.0;
                }
            }

            // --- FLIGHT CONTROL LOOP (PID) ---
            std::array<double, 3> action;
            if (phase == "TAKEOFF ROLL" || (phase == "LANDING/BRAKING" && alt < 0.5)) {
                // Ground phase: bypass altitude PID to prevent integral windup from vertical forces
                // Directly control pitch and throttle
                double u_throttle = (speed_ref > speed) ? 1.0 : 0.0;
                if (phase == "LANDING/BRAKING") u_throttle = 0.0; // cut engine
                
                double u_elevator = -1.0 * (pitch_ref - pitch); // proportional pitch hold
                action = {u_elevator, 0.0, u_throttle};
            } else {
                // Airborne phase: Full 3-axis PID control
                // Compute dynamic pitch reference from altitude error
                double alt_error = alt_ref - alt;
                double dynamic_pitch_ref = std::clamp(0.08 * alt_error, -0.22, 0.22);
                
                pid.set_reference(roll_ref, dynamic_pitch_ref, speed_ref);
                action = pid.get_action(roll, pitch, sim.state["Va"]->value,
                                        {sim.state["omega_p"]->value, sim.state["omega_q"]->value, sim.state["omega_r"]->value});
            }

            // Command vector: [elevator, aileron, throttle, gear_cmd]
            std::vector<double> commands = {action[0], action[1], action[2], gear_cmd};
            
            std::string term;
            sim.step(commands, term);

            // Log to CSV
            csv << i << ","
                << roll << "," << pitch << "," << sim.state["yaw"]->value << ","
                << sim.state["position_n"]->value << "," << sim.state["position_e"]->value << "," << sim.state["position_d"]->value << ","
                << speed << "," << sim.state["Va"]->value << "," << sim.state["gear_deploy"]->value << ","
                << sim.state["throttle"]->value << "," << sim.state["elevator"]->value << "\n";

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

            // Send MAVLink to QGC
            if (i % 2 == 0) {
                uint32_t time_ms = static_cast<uint32_t>(t * 1000.0);
                mavlink.send_heartbeat();
                mavlink.send_attitude(time_ms, roll, pitch, sim.state["yaw"]->value,
                                      sim.state["omega_p"]->value, sim.state["omega_q"]->value, sim.state["omega_r"]->value);
                
                double lat = 40.128 + sim.state["position_n"]->value / 111111.0;
                double lon = 32.995 + sim.state["position_e"]->value / (111111.0 * std::cos(40.128 * M_PI / 180.0));
                double alt = 950.0 - sim.state["position_d"]->value;
                double rel_alt = -sim.state["position_d"]->value;
                
                mavlink.send_global_position(time_ms, lat, lon, alt, rel_alt, speed, 0.0f, 0.0f, sim.state["yaw"]->value * 180.0 / M_PI);
            }

            usleep(10000); // 10ms wait (100Hz)
        }

        csv.close();
        close(vis_socket);
        std::cout << "Scenario 3 Complete. Trajectory saved to scenario3_trajectory.csv" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
