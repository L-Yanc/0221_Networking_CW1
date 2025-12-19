#include "attacks.hpp"

extern "C" {
#include "esp_log.h"
#include "esp_timer.h"
}

#include <cstring>
#include <vector>
#include <algorithm>

#include "config.hpp"
#include "comms.hpp"

namespace attacks {

    static const char* TAG = "ATTACKS";

    static AttackMode s_mode = AttackMode::None;

    struct ReplayBufferEntry {
        comms::LoraPacket pkt{};
        bool in_use = false;
    };

    static constexpr int REPLAY_BUFFER_SIZE = 16;
    static ReplayBufferEntry s_replay_buffer[REPLAY_BUFFER_SIZE];
    static int s_replay_index = 0;

    static uint32_t s_last_fasttx_ms = 0;

    // --------------------------------------------------------
    // Helper: convert mode to string
    // --------------------------------------------------------

    static const char* mode_to_string(AttackMode mode)
    {
        switch (mode) {
        case AttackMode::None:
            return "None";
        case AttackMode::Replay:
            return "Replay";
        case AttackMode::Ghost:
            return "Ghost";
        case AttackMode::FalseData:
            return "FalseData";
        case AttackMode::FastTx:
            return "FastTx";
        case AttackMode::InvalidCmac:
            return "InvalidCmac";
        default:
            return "Unknown";
        }
    }

    // --------------------------------------------------------
    // Mode control
    // --------------------------------------------------------

    void set_attack_mode(AttackMode mode)
    {
        s_mode = mode;
        ESP_LOGI(TAG, "Attack mode set to %s", mode_to_string(mode));
    }

    AttackMode get_attack_mode()
    {
        return s_mode;
    }

    AttackMode next_attack_mode()
    {
        int m = static_cast<int>(s_mode);
        m = (m + 1) % 6;
        s_mode = static_cast<AttackMode>(m);
        ESP_LOGI(TAG, "Attack mode cycled to %s", mode_to_string(s_mode));
        return s_mode;
    }

    // --------------------------------------------------------
    // Replay buffer helpers
    // --------------------------------------------------------

    static void replay_buffer_add(const comms::LoraPacket& pkt)
    {
        s_replay_buffer[s_replay_index].pkt = pkt;
        s_replay_buffer[s_replay_index].in_use = true;
        s_replay_index = (s_replay_index + 1) % REPLAY_BUFFER_SIZE;
    }

    static bool replay_buffer_get_any(comms::LoraPacket& out_pkt)
    {
        for (int i = 0; i < REPLAY_BUFFER_SIZE; ++i) {
            if (s_replay_buffer[i].in_use) {
                out_pkt = s_replay_buffer[i].pkt;
                return true;
            }
        }
        return false;
    }

    // --------------------------------------------------------
    // Individual attack implementations
    // --------------------------------------------------------

    // NONE: do nothing
    static void apply_none(comms::LoraPacket& pkt, uint32_t now_ms)
    {
        (void)pkt;
        (void)now_ms;
    }

    // REPLAY: replace outgoing packet with a stored old one
    static void apply_replay(comms::LoraPacket& pkt, uint32_t now_ms)
    {
        (void)now_ms;

        // Save current packet into replay buffer for future reuse
        replay_buffer_add(pkt);

        comms::LoraPacket old_pkt;
        if (replay_buffer_get_any(old_pkt)) {
            // Use an old packet as the outgoing content
            pkt = old_pkt;

            pkt.ts_ms ^= 0x007F;
        }
    }

    // GHOST: fabricate a fake drone based on neighbour info
    static void apply_ghost(comms::LoraPacket& pkt, uint32_t now_ms)
    {
        // Basic idea:
        //  - get neighbour snapshot
        //  - choose one real drone and offset its position
        //  - use a fake node_id for the ghost drone
        //  - keep team, version, and realistic velocity
        auto neighbours = comms::get_neighbour_snapshot(now_ms);
        if (neighbours.empty()) {
            return; // nothing to ghost from
        }

        const auto& base = neighbours[0]; // could randomise later

        // Make up a fake node_id (should not collide with real)
        uint8_t fake_id[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};
        std::memcpy(pkt.node_id, fake_id, sizeof(fake_id));

        // Copy base movement but shift position out of plausible range
        // Here we go just outside the valid world cube
        pkt.x_mm = WORLD_MAX_MM + 1000; // definitely outside 100m cube
        pkt.y_mm = base.y_mm;
        pkt.z_mm = base.z_mm;

        pkt.vx_mm_s = base.vx_mm_s;
        pkt.vy_mm_s = base.vy_mm_s;
        pkt.vz_mm_s = base.vz_mm_s;

        // Keep yaw_cd from base for realism
        pkt.yaw_cd = base.yaw_cd;
    }

    // FALSE DATA: keep node_id + CMAC valid, but push state into crazy values
    static void apply_false_data(comms::LoraPacket& pkt, uint32_t now_ms)
    {
        (void)now_ms;

        // Teleport to a corner of the cube
        pkt.x_mm = WORLD_MAX_MM;
        pkt.y_mm = WORLD_MAX_MM;
        pkt.z_mm = WORLD_MAX_MM;

        // Unrealistic velocities (well beyond MAX_VELOCITY_MM_S)
        pkt.vx_mm_s = MAX_VELOCITY_MM_S * 10;
        pkt.vy_mm_s = MAX_VELOCITY_MM_S * 10;
        pkt.vz_mm_s = MAX_VELOCITY_MM_S * 10;
    }

    // FAST TX: nothing to mutate in pkt here; handled in should_send_this_tick()
    static void apply_fast_tx(comms::LoraPacket& pkt, uint32_t now_ms)
    {
        (void)pkt;
        (void)now_ms;
        // No direct packet mutation for this mode
    }

    // INVALID CMAC:
    //
    // Because attacks::apply_attacks() runs BEFORE CMAC is computed in comms::send_packet(),
    // we *cannot* directly corrupt mac_tag here (it will be overwritten).
    //
    // To truly implement this:
    //   - add a small hook in comms::send_packet() to check get_attack_mode()
    //   - if AttackMode::InvalidCmac, flip some bits in pkt.mac_tag before TX
    //
    // Here we just leave the state unchanged; the "invalid" behaviour is implemented
    // in comms.cpp once you add that hook.
    static void apply_invalid_cmac(comms::LoraPacket& pkt, uint32_t now_ms)
    {
        (void)pkt;
        (void)now_ms;
        // Real CMAC corruption happens in comms::send_packet() via get_attack_mode().
    }

    // --------------------------------------------------------
    // Public API
    // --------------------------------------------------------

    void apply_attacks(
        AttackMode mode,
        comms::LoraPacket& pkt,
        uint32_t now_ms
    )
    {
        const int64_t t_start_us = esp_timer_get_time();
        
        switch (mode) {
        case AttackMode::None:
            apply_none(pkt, now_ms);
            break;
        case AttackMode::Replay:
#if ENABLE_ATTACK_REPLAY
            apply_replay(pkt, now_ms);
            ESP_LOGI(TAG, "ATTACK Replay applied");
#endif
            break;
        case AttackMode::Ghost:
#if ENABLE_ATTACK_GHOST
            apply_ghost(pkt, now_ms);
            ESP_LOGI(TAG, "ATTACK Ghost applied");
#endif
            break;
        case AttackMode::FalseData:
#if ENABLE_ATTACK_FALSE_DATA
            apply_false_data(pkt, now_ms);
            ESP_LOGI(TAG, "ATTACK FalseData applied");
#endif
            break;
        case AttackMode::FastTx:
#if ENABLE_ATTACK_FAST_TX
            apply_fast_tx(pkt, now_ms);
#endif
            break;
        case AttackMode::InvalidCmac:
#if ENABLE_ATTACK_INVALID_CMAC
            apply_invalid_cmac(pkt, now_ms);
            ESP_LOGI(TAG, "ATTACK InvalidCmac applied");
#endif
            break;
        default:
            apply_none(pkt, now_ms);
            break;
        }
        
        const int64_t t_end_us = esp_timer_get_time();
        if (mode != AttackMode::None) {
            ESP_LOGI(TAG, "ATTACK %s exec_us=%lld",
                    mode_to_string(mode), (long long)(t_end_us - t_start_us));
        }
    }

    bool should_send_this_tick(
        AttackMode mode,
        uint32_t now_ms
    )
    {
        if (mode != AttackMode::FastTx) {
            // Normal behaviour: rely on RADIO_HZ schedule from radio_task
            return true;
        }

        // FastTx example:
        // - radio_task is already at RADIO_HZ
        // - we further allow sending at most once every 100 ms
        //   (you can tweak this based on your duty-cycle constraints)
        if (now_ms - s_last_fasttx_ms < 100) {
            return false;
        }

        s_last_fasttx_ms = now_ms;
        ESP_LOGI(TAG, "ATTACK FastTx packet sent");
        return true;
    }

} // namespace attacks
