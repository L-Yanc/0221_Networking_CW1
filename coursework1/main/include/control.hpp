// control.hpp

#pragma once

#include <cstdint>
#include <vector>

#include "config.hpp"
#include "comms.hpp"   // for NeighbourEntry

namespace control {

    struct LocalState {
        uint32_t x_mm = 50000;
        uint32_t y_mm = 50000;
        uint32_t z_mm = 50000;

        int32_t vx_mm_s = 0;
        int32_t vy_mm_s = 0;
        int32_t vz_mm_s = 0;

        uint16_t yaw_cd = 0;
        uint16_t seq_number = 0;
    };

    struct ControlCmd {
        int32_t vx_cmd_mm_s = 0;
        int32_t vy_cmd_mm_s = 0;
        int32_t vz_cmd_mm_s = 0;

        int32_t yaw_rate_cd_s = 0;
    };

    void clamp_state(LocalState& st);

    void physics_step(LocalState& state, const ControlCmd& cmd, float dt_s);

    struct FlockMetrics {
        float min_separation_mm = 0.0f;
        float avg_distance_to_center = 0.0f;
        float heading_alignment = 0.0f;
        int neighbour_count = 0;
    };

    ControlCmd compute_flock_command(
        const LocalState& self,
        const std::vector<comms::NeighbourEntry>& neighbours
    );

} // namespace control
