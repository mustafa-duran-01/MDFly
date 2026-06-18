#ifndef MDFLY_MAVLINK_COMM_H
#define MDFLY_MAVLINK_COMM_H

#include <string>
#include <vector>
#include <array>
#include <cstdint>

namespace mdfly {

class MavlinkComm {
public:
    MavlinkComm();
    ~MavlinkComm();

    // Initialize UDP connection to target IP and port (e.g. 127.0.0.1:14550)
    bool init(const std::string& ip = "127.0.0.1", int port = 14550);

    // Send MAVLink packets
    void send_heartbeat();
    
    void send_attitude(uint32_t time_ms, float roll, float pitch, float yaw,
                       float rollspeed, float pitchspeed, float yawspeed);
                       
    void send_global_position(uint32_t time_ms, double lat, double lon, double alt_m,
                             double rel_alt_m, float vx_mps, float vy_mps, float vz_mps, float hdg_deg);

private:
    int socket_fd = -1;
    void* sockaddr_dest = nullptr; // Abstracted sockaddr_in to avoid including netinet/in.h in header
    uint8_t seq = 0;

    void send_packet(uint8_t msg_id, const uint8_t* payload, uint8_t length, uint8_t crc_extra);
};

} // namespace mdfly

#endif // MDFLY_MAVLINK_COMM_H
