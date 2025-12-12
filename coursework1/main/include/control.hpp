// control.hpp
//
// Control logic for COMP0221 CW
// - Local drone state representation
// - Control command representation
// - Physics integration step (50 Hz)
// - Flocking command generation (Reynolds rules)

#pragma once

#include <cstdint>
#include <vector>

#include "config.hpp"
#include "comms.hpp"   // for NeighbourEntry

namespace control {

    // --------------------------------------------------------
    // Local drone state (our own drone)
    // --------------------------------------------------------
    struct LocalState {
        uint32_t x_mm   = 50000;  // start somewhere in the middle of the 100m cube
        uint32_t y_mm   = 50000;
        uint32_t z_mm   = 50000;

        int32_t  vx_mm_s = 0;
        int32_t  vy_mm_s = 0;
        int32_t  vz_mm_s = 0;

        uint16_t yaw_cd  = 0;     // heading in centidegrees (0..35999)
        uint16_t seq_number = 0;  // local sequence counter for packets
    };

    // --------------------------------------------------------
    // Control command
    // Physics will try to track these velocity commands
    // --------------------------------------------------------
    struct ControlCmd {
        int32_t vx_cmd_mm_s = 0;
        int32_t vy_cmd_mm_s = 0;
        int32_t vz_cmd_mm_s = 0;

        int32_t yaw_rate_cd_s = 0;    // centidegrees per second
    };


    // --------------------------------------------------------
    // Physics
    // --------------------------------------------------------

    // Clamp position and velocity to world and speed limits
    void clamp_state(LocalState& st);

    // Single physics integration step.
    //
    // dt_s   = timestep in seconds (e.g. 1.0f / 50.0f for 50 Hz)
    // state  = in/out, will be updated in-place
    // cmd    = desired velocities and yaw rate
    //
    // Typical behaviour:
    //   - optionally low-pass filter velocities toward cmd
    //   - integrate position: x += vx * dt
    //   - integrate yaw: yaw += yaw_rate * dt
    //   - clamp to WORLD_MIN/WORLD_MAX and MAX_VELOCITY_MM_S
    void physics_step(LocalState& state, const ControlCmd& cmd, float dt_s);


    // --------------------------------------------------------
    // Flocking
    // --------------------------------------------------------

    // Compute a flocking-based control command based on:
    //  - our current state
    //  - a snapshot of neighbours from comms::get_neighbour_snapshot()
    //
    // neighbours: vector<NeighbourEntry> (positions, velocities, yaw, timestamps)
    //
    // This will implement Reynolds style:
    //  - separation
    //  - cohesion
    //  - alignment
    // and yaw alignment toward resulting velocity direction.
    ControlCmd compute_flock_command(
        const LocalState& self,
        const std::vector<comms::NeighbourEntry>& neighbours
    );

    // Optional: helper for computing stability metrics later
    struct FlockMetrics {
        float min_separation_mm      = 0.0f;
        float avg_distance_to_center = 0.0f;
        float heading_alignment      = 0.0f;   // e.g. cosine similarity
        int   neighbour_count        = 0;
    };

    // Compute metrics for logging / telemetry.
    // You can call this from telemetry_task if you want graphs later.
    FlockMetrics compute_flock_metrics(
        const LocalState& self,
        const std::vector<comms::NeighbourEntry>& neighbours
    );

} // namespace control
