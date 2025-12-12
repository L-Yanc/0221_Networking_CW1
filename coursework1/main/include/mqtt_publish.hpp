// mqtt_publish.hpp
//
// Simple MQTT publisher for telemetry

#pragma once

#include <cstdint>

namespace mqtt_pub {

    // Initialize MQTT client (call once from main)
    void init();

    // Publish a telemetry JSON packet
    // format: {"version":1,"team_id":0,"node_id":"AA:BB:CC:DD:EE:FF",...}
    void publish_telemetry(const char* json_payload);

    // Check if MQTT is connected
    bool is_connected();

} // namespace mqtt_pub
