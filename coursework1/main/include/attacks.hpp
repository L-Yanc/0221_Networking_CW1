// attacks.hpp
//
// Attack controller for COMP0221 CW
// - Central place to enable/disable attack modes
// - Provides hooks for mutating outgoing packets
// - Some modes may need access to neighbour snapshots (via comms)
//
// This operates on LoraPacket *before* CMAC is computed and send_packet() is called.

#pragma once

#include <cstdint>

#include "config.hpp"
#include "comms.hpp"

namespace attacks {

    // --------------------------------------------------------
    // Global attack mode control
    // --------------------------------------------------------

    // Set the global attack mode (e.g. from a CLI, button, or compile-time)
    void set_attack_mode(AttackMode mode);

    // Get current attack mode
    AttackMode get_attack_mode();

    // Optional: cycle through modes (for quick testing)
    AttackMode next_attack_mode();


    // --------------------------------------------------------
    // Attack application
    // --------------------------------------------------------

    void apply_attacks(
        AttackMode mode,
        comms::LoraPacket& pkt,
        uint32_t now_ms
    );

    // --------------------------------------------------------
    // Fast TX helper
    // --------------------------------------------------------

    bool should_send_this_tick(
        AttackMode mode,
        uint32_t now_ms
    );

} // namespace attacks
