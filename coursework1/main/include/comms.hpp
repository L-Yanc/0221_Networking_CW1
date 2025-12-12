// comms.hpp
//
// Communication layer for COMP0221 CW
// - LoRa packet struct (on air format)
// - CMAC helpers
// - Neighbour table API
// - Radio init and send entry points

#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

#include "config.hpp"

namespace comms {

    // --------------------------------------------------------
    // LoRa packet format
    // Matches COMP0221 Standard (46 bytes total)
    // --------------------------------------------------------
#pragma pack(push, 1)
    struct LoraPacket {
        uint8_t  version;
        uint8_t  team_id;
        uint8_t  node_id[6];
        uint16_t seq_number;
        uint32_t ts_s;
        uint16_t ts_ms;
        uint32_t x_mm;
        uint32_t y_mm;
        uint32_t z_mm;
        int32_t  vx_mm_s;
        int32_t  vy_mm_s;
        int32_t  vz_mm_s;
        uint16_t yaw_cd;
        uint8_t  mac_tag[MAC_TAG_LEN];   // 4 byte truncated CMAC
    };
#pragma pack(pop)

    static_assert(sizeof(LoraPacket) == PACKET_LEN_BYTES, "LoraPacket size must be 46 bytes");

    // --------------------------------------------------------
    // Neighbour representation
    // --------------------------------------------------------
    struct NeighbourEntry {
        uint8_t  node_id[6] {};
        uint32_t x_mm       = 0;
        uint32_t y_mm       = 0;
        uint32_t z_mm       = 0;
        int32_t  vx_mm_s    = 0;
        int32_t  vy_mm_s    = 0;
        int32_t  vz_mm_s    = 0;
        uint16_t yaw_cd     = 0;

        uint16_t seq_number = 0;
        uint32_t last_seen_ms = 0;
        bool     in_use     = false;
    };

    // --------------------------------------------------------
    // Radio / LoRa control
    // --------------------------------------------------------

    // Set up the SX127x Radio driver, register callbacks, start continuous RX
    void radio_init();

    // Explicitly restart RX mode if needed
    void start_radio_rx();

    // High level send:
    // - takes a fully populated LoraPacket (mac_tag may be set or not)
    // - computes CMAC if needed
    // - serialises and hands off to the low level driver
    bool send_packet(const LoraPacket& pkt);


    // --------------------------------------------------------
    // Packet <-> bytes and CMAC
    // --------------------------------------------------------

    // Serialise to raw bytes for transmission
    // buf must be at least PACKET_LEN_BYTES long
    void pack_to_bytes(const LoraPacket& pkt, uint8_t* buf_out);

    // Parse from raw bytes into a packet struct
    // Returns false if length is wrong or obviously invalid
    bool unpack_from_bytes(const uint8_t* buf, size_t len, LoraPacket& pkt_out);

    // Compute CMAC over 'len' bytes in 'data' using TEAM_KEY
    // Writes MAC_TAG_LEN bytes into 'tag_out'
    bool compute_cmac(const uint8_t* data, size_t len, uint8_t tag_out[MAC_TAG_LEN]);

    // Verify that 'tag' matches the CMAC over data
    bool verify_cmac(const uint8_t* data, size_t len, const uint8_t tag[MAC_TAG_LEN]);


    // --------------------------------------------------------
    // Neighbour table API
    // --------------------------------------------------------

    // Update or insert neighbour based on an incoming packet
    // now_ms is time since boot in milliseconds
    void update_neighbour_from_packet(const LoraPacket& pkt, uint32_t now_ms);

    // Remove neighbours that have not been heard from for NEIGHBOUR_TIMEOUT_MS
    void cull_stale_neighbours(uint32_t now_ms);

    // Get a snapshot of all current neighbours
    // This is a copy so caller can iterate without holding a lock
    std::vector<NeighbourEntry> get_neighbour_snapshot(uint32_t now_ms);

} // namespace comms
