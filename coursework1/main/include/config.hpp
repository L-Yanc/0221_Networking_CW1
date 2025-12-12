// config.hpp
//
// Global compile-time configuration for the COMP0221 CW project.
// Contains protocol constants, world limits, timings, LoRa parameters,
// flocking weights, neighbour settings, crypto key declarations, etc.

#pragma once
#include <cstdint>
#include <cstddef>

// ------------------------------------------------------------
// Team + Protocol
// ------------------------------------------------------------
inline constexpr uint8_t TEAM_ID            = 0;
inline constexpr uint8_t PROTOCOL_VERSION   = 1;

// ------------------------------------------------------------
// World Space (100m × 100m × 100m)
// ------------------------------------------------------------
inline constexpr int32_t WORLD_MIN_MM       = 0;
inline constexpr int32_t WORLD_MAX_MM       = 100000;   // 100m

// ------------------------------------------------------------
// Task Frequencies (Hz)
// ------------------------------------------------------------
inline constexpr int PHYSICS_HZ             = 50;       // 50 Hz
inline constexpr int FLOCK_HZ               = 10;       // 10 Hz
inline constexpr int RADIO_HZ               = 4;        // 4 Hz
inline constexpr int TELEMETRY_HZ           = 2;        // 2 Hz

// ------------------------------------------------------------
// Flocking Weights (match your friend's defaults)
// These will be tuned later if needed
// ------------------------------------------------------------
inline constexpr float ALIGNMENT_WEIGHT     = 0.02f;
inline constexpr float COHESION_WEIGHT      = 0.05f;
inline constexpr float SEPARATION_WEIGHT    = 0.15f;

inline constexpr int32_t MIN_SEPARATION_MM  = 5000;     // 5m
inline constexpr int32_t PERCEPTION_RADIUS_MM = 30000;  // 30m

inline constexpr int32_t MAX_VELOCITY_MM_S  = 500;      // mm/s

// ------------------------------------------------------------
// Neighbour Table Limits
// ------------------------------------------------------------
inline constexpr int MAX_NEIGHBOURS         = 10;
inline constexpr uint32_t NEIGHBOUR_TIMEOUT_MS = 10000;  // 5 seconds before stale

// Individual Attack Toggles (set to 1 to enable, 0 to disable)

// ------------------------------------------------------------
// LoRa Configuration (based on the COMP0220 Standard document)
// ------------------------------------------------------------
inline constexpr uint32_t LORA_FREQ_HZ      = 868100000;  // 868.1 MHz
inline constexpr uint8_t  LORA_SF           = 9;
inline constexpr uint8_t  LORA_BW           = 250;        // kHz
inline constexpr uint8_t  LORA_CR           = 7;          // 4/7
inline constexpr uint8_t  LORA_PREAMBLE_LEN = 10;
inline constexpr uint8_t  LORA_SYNCWORD     = 0x12;
inline constexpr int8_t   LORA_TX_POWER_DBM = 14;

// ------------------------------------------------------------
// Packet Sizes
// (COMP0221 Standard = 46 bytes including 4-byte CMAC tag)
// ------------------------------------------------------------
inline constexpr size_t PACKET_LEN_BYTES    = 46;
inline constexpr size_t MAC_TAG_LEN         = 4;

// ------------------------------------------------------------
// Crypto Key (AES-128 CMAC)
// DO NOT COMMIT YOUR REAL KEY TO PUBLIC REPOS
//
// Declared here, defined privately in comms.cpp:
//   const uint8_t TEAM_KEY[16] = { ... };
// ------------------------------------------------------------
extern const uint8_t TEAM_KEY[16];

// ------------------------------------------------------------
// Attack Mode Enum (shared by tasks + attacks modules)
// ------------------------------------------------------------
enum class AttackMode : uint8_t {
    None,
    Replay,
    Ghost,
    FalseData,
    FastTx,
    InvalidCmac,
};
