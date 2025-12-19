// mqtt_publish.hpp
//
// Simple MQTT publisher for telemetry

#pragma once

#include <cstdint>
#include <cstddef>

namespace mqtt_pub {

    // Initialize MQTT client (call once from main)
    void init();

    // Publish a telemetry JSON packet (non-blocking)
    void publish_telemetry(const char* json_payload);

    // Build JSON telemetry from LocalState and publish (non-blocking)
    // Returns false if MQTT not connected
    bool publish_state(uint32_t x_mm, uint32_t y_mm, uint32_t z_mm,
                       int32_t vx_mm_s, int32_t vy_mm_s, int32_t vz_mm_s,
                       uint16_t yaw_cd, uint16_t seq_number);

    // Check if MQTT is connected
    bool is_connected();

} // namespace mqtt_pub
