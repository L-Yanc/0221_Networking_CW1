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

    // Apply attack logic to an outgoing packet in-place.
    //
    // - mode      : current attack mode
    // - pkt       : packet about to be sent (fields can be modified)
    // - now_ms    : current time since boot in ms (esp_timer_get_time()/1000)
    //
    // This is called from radio_task BEFORE comms::send_packet(pkt).
    void apply_attacks(
        AttackMode mode,
        comms::LoraPacket& pkt,
        uint32_t now_ms
    );

    // --------------------------------------------------------
    // Fast TX helper
    // --------------------------------------------------------

    // Some attacks (e.g. FastTx) might want to send more often than RADIO_HZ.
    // You can use this helper in radio_task to decide whether to skip or send.
    //
    // Returns true if we should send a packet at this tick, false if we should skip.
    bool should_send_this_tick(
        AttackMode mode,
        uint32_t now_ms
    );

} // namespace attacks
