#include "mdfly/mavlink_comm.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace mdfly {

#pragma pack(push, 1)
struct HeartbeatPayload {
    uint32_t custom_mode = 0;
    uint8_t type = 1;         // MAV_TYPE_FIXED_WING
    uint8_t autopilot = 12;    // MAV_AUTOPILOT_PX4
    uint8_t base_mode = 81;    // MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED
    uint8_t system_status = 4; // MAV_STATE_ACTIVE
    uint8_t mavlink_version = 3;
};

struct AttitudePayload {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
};

struct GlobalPositionPayload {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg;
};
#pragma pack(pop)

MavlinkComm::MavlinkComm() {
    sockaddr_dest = malloc(sizeof(sockaddr_in));
    memset(sockaddr_dest, 0, sizeof(sockaddr_in));
}

MavlinkComm::~MavlinkComm() {
    if (socket_fd != -1) {
        close(socket_fd);
    }
    if (sockaddr_dest) {
        free(sockaddr_dest);
    }
}

bool MavlinkComm::init(const std::string& ip, int port) {
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        std::cerr << "Failed to create MAVLink UDP socket" << std::endl;
        return false;
    }

    sockaddr_in* dest = static_cast<sockaddr_in*>(sockaddr_dest);
    dest->sin_family = AF_INET;
    dest->sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &dest->sin_addr) <= 0) {
        std::cerr << "Invalid target IP address for MAVLink" << std::endl;
        return false;
    }

    return true;
}

static void crc_accumulate(uint8_t data, uint16_t& crc) {
    uint8_t tmp = data ^ (uint8_t)(crc & 0xff);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void MavlinkComm::send_packet(uint8_t msg_id, const uint8_t* payload, uint8_t length, uint8_t crc_extra) {
    if (socket_fd == -1) return;

    // Header size (6) + Payload + CRC (2)
    std::vector<uint8_t> buffer(6 + length + 2);
    buffer[0] = 0xFE;   // Start sign
    buffer[1] = length; // Payload length
    buffer[2] = seq++;  // Packet sequence
    buffer[3] = 1;      // System ID
    buffer[4] = 1;      // Component ID
    buffer[5] = msg_id; // Message ID

    // Copy payload
    std::memcpy(&buffer[6], payload, length);

    // Compute checksum
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 1; i < 6 + length; ++i) {
        crc_accumulate(buffer[i], crc);
    }
    crc_accumulate(crc_extra, crc);

    buffer[6 + length] = static_cast<uint8_t>(crc & 0xFF);
    buffer[6 + length + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    sockaddr_in* dest = static_cast<sockaddr_in*>(sockaddr_dest);
    sendto(socket_fd, buffer.data(), buffer.size(), 0, (struct sockaddr*)dest, sizeof(sockaddr_in));
}

void MavlinkComm::send_heartbeat() {
    HeartbeatPayload payload;
    send_packet(0, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload), 50);
}

void MavlinkComm::send_attitude(uint32_t time_ms, float roll, float pitch, float yaw,
                               float rollspeed, float pitchspeed, float yawspeed) {
    AttitudePayload payload;
    payload.time_boot_ms = time_ms;
    payload.roll = roll;
    payload.pitch = pitch;
    payload.yaw = yaw;
    payload.rollspeed = rollspeed;
    payload.pitchspeed = pitchspeed;
    payload.yawspeed = yawspeed;
    send_packet(30, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload), 39);
}

void MavlinkComm::send_global_position(uint32_t time_ms, double lat, double lon, double alt_m,
                                     double rel_alt_m, float vx_mps, float vy_mps, float vz_mps, float hdg_deg) {
    GlobalPositionPayload payload;
    payload.time_boot_ms = time_ms;
    payload.lat = static_cast<int32_t>(lat * 1e7);
    payload.lon = static_cast<int32_t>(lon * 1e7);
    payload.alt = static_cast<int32_t>(alt_m * 1000.0);
    payload.relative_alt = static_cast<int32_t>(rel_alt_m * 1000.0);
    payload.vx = static_cast<int16_t>(vx_mps * 100.0f);
    payload.vy = static_cast<int16_t>(vy_mps * 100.0f);
    payload.vz = static_cast<int16_t>(vz_mps * 100.0f);
    
    // Normalize heading to [0, 360] then to [0, 36000] cdeg
    float hdg = hdg_deg;
    while (hdg < 0.0f) hdg += 360.0f;
    while (hdg >= 360.0f) hdg -= 360.0f;
    payload.hdg = static_cast<uint16_t>(hdg * 100.0f);

    send_packet(33, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload), 104);
}

} // namespace mdfly
