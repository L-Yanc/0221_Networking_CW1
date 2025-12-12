#include "control.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "config.hpp"
#include "comms.hpp"

namespace control {

    static float clamp_float(float v, float lo, float hi)
    {
        return std::max(lo, std::min(hi, v));
    }

    static int32_t clamp_int32(int32_t v, int32_t lo, int32_t hi)
    {
        return std::max(lo, std::min(hi, v));
    }

    // Wrap yaw into [0, 36000) centidegrees
    static uint16_t wrap_yaw_cd(int32_t yaw_cd)
    {
        int32_t y = yaw_cd % 36000;
        if (y < 0) {
            y += 36000;
        }
        return static_cast<uint16_t>(y);
    }

    static int32_t yaw_error_cd(uint16_t current_cd, uint16_t target_cd)
    {
        int32_t err = static_cast<int32_t>(target_cd) - static_cast<int32_t>(current_cd);
        while (err > 18000) err -= 36000;
        while (err < -18000) err += 36000;
        return err;
    }

    void clamp_state(LocalState& st)
    {
        st.x_mm = static_cast<uint32_t>(
            clamp_int32(static_cast<int32_t>(st.x_mm), WORLD_MIN_MM, WORLD_MAX_MM)
        );
        st.y_mm = static_cast<uint32_t>(
            clamp_int32(static_cast<int32_t>(st.y_mm), WORLD_MIN_MM, WORLD_MAX_MM)
        );
        st.z_mm = static_cast<uint32_t>(
            clamp_int32(static_cast<int32_t>(st.z_mm), WORLD_MIN_MM, WORLD_MAX_MM)
        );

        float vx = static_cast<float>(st.vx_mm_s);
        float vy = static_cast<float>(st.vy_mm_s);
        float vz = static_cast<float>(st.vz_mm_s);

        float speed = std::sqrt(vx * vx + vy * vy + vz * vz);

        if (speed > static_cast<float>(MAX_VELOCITY_MM_S) && speed > 0.0f) {
            float scale = static_cast<float>(MAX_VELOCITY_MM_S) / speed;
            vx *= scale;
            vy *= scale;
            vz *= scale;
            st.vx_mm_s = static_cast<int32_t>(vx);
            st.vy_mm_s = static_cast<int32_t>(vy);
            st.vz_mm_s = static_cast<int32_t>(vz);
        }

        st.yaw_cd = wrap_yaw_cd(st.yaw_cd);
    }

    void physics_step(LocalState& state, const ControlCmd& cmd, float dt_s)
    {
        // 1) Move velocities toward commanded values (simple first-order response)
        const float vel_track_gain = 1.0f; // 1.0 = instant; <1.0 = smoother

        float vx     = static_cast<float>(state.vx_mm_s);
        float vy     = static_cast<float>(state.vy_mm_s);
        float vz     = static_cast<float>(state.vz_mm_s);

        float vx_cmd = static_cast<float>(cmd.vx_cmd_mm_s);
        float vy_cmd = static_cast<float>(cmd.vy_cmd_mm_s);
        float vz_cmd = static_cast<float>(cmd.vz_cmd_mm_s);

        vx += vel_track_gain * (vx_cmd - vx);
        vy += vel_track_gain * (vy_cmd - vy);
        vz += vel_track_gain * (vz_cmd - vz);

        state.vx_mm_s = static_cast<int32_t>(vx);
        state.vy_mm_s = static_cast<int32_t>(vy);
        state.vz_mm_s = static_cast<int32_t>(vz);

        // 2) Integrate position
        state.x_mm = static_cast<uint32_t>(
            clamp_int32(
                static_cast<int32_t>(state.x_mm) +
                    static_cast<int32_t>(vx * dt_s),
                WORLD_MIN_MM,
                WORLD_MAX_MM
            )
        );
        state.y_mm = static_cast<uint32_t>(
            clamp_int32(
                static_cast<int32_t>(state.y_mm) +
                    static_cast<int32_t>(vy * dt_s),
                WORLD_MIN_MM,
                WORLD_MAX_MM
            )
        );
        state.z_mm = static_cast<uint32_t>(
            clamp_int32(
                static_cast<int32_t>(state.z_mm) +
                    static_cast<int32_t>(vz * dt_s),
                WORLD_MIN_MM,
                WORLD_MAX_MM
            )
        );

        // 3) Integrate yaw
        int32_t yaw = static_cast<int32_t>(state.yaw_cd);
        yaw += static_cast<int32_t>(static_cast<float>(cmd.yaw_rate_cd_s) * dt_s);
        state.yaw_cd = wrap_yaw_cd(yaw);

        // 4) Final clamp
        clamp_state(state);
    }

    // --------------------------------------------------------
    // Flocking
    // --------------------------------------------------------

    ControlCmd compute_flock_command(
        const LocalState& self,
        const std::vector<comms::NeighbourEntry>& neighbours
    )
    {
        ControlCmd cmd{};

        if (neighbours.empty()) {
            // no neighbours → stay put by default
            return cmd;
        }

        // Accumulators
        float align_vx = 0.0f;
        float align_vy = 0.0f;
        float align_vz = 0.0f;

        float center_x = 0.0f;
        float center_y = 0.0f;
        float center_z = 0.0f;

        float sep_x = 0.0f;
        float sep_y = 0.0f;
        float sep_z = 0.0f;

        int count = 0;

        const float min_sep_sq     = static_cast<float>(MIN_SEPARATION_MM) *
                                     static_cast<float>(MIN_SEPARATION_MM);
        const float perception_sq  = static_cast<float>(PERCEPTION_RADIUS_MM) *
                                     static_cast<float>(PERCEPTION_RADIUS_MM);

        for (const auto& n : neighbours) {
            float dx = static_cast<float>(n.x_mm) - static_cast<float>(self.x_mm);
            float dy = static_cast<float>(n.y_mm) - static_cast<float>(self.y_mm);
            float dz = static_cast<float>(n.z_mm) - static_cast<float>(self.z_mm);

            float dist_sq = dx * dx + dy * dy + dz * dz;

            if (dist_sq > perception_sq) {
                continue;
            }

            ++count;

            // Alignment: average neighbour velocity
            align_vx += static_cast<float>(n.vx_mm_s);
            align_vy += static_cast<float>(n.vy_mm_s);
            align_vz += static_cast<float>(n.vz_mm_s);

            // Cohesion: average neighbour position
            center_x += static_cast<float>(n.x_mm);
            center_y += static_cast<float>(n.y_mm);
            center_z += static_cast<float>(n.z_mm);

            // Separation: push away from close neighbours (not normalized)
            if (dist_sq < min_sep_sq && dist_sq > 0.0f) {
                // Use dx/dy/dz directly (not normalized) - matches friend's code
                sep_x -= dx;
                sep_y -= dy;
                sep_z -= dz;
            }
        }

        if (count == 0) {
            // no neighbours in perception radius
            return cmd;
        }

        // Compute averages
        align_vx /= static_cast<float>(count);
        align_vy /= static_cast<float>(count);
        align_vz /= static_cast<float>(count);

        center_x /= static_cast<float>(count);
        center_y /= static_cast<float>(count);
        center_z /= static_cast<float>(count);

        // Cohesion vector: toward centroid
        float coh_x = center_x - static_cast<float>(self.x_mm);
        float coh_y = center_y - static_cast<float>(self.y_mm);
        float coh_z = center_z - static_cast<float>(self.z_mm);

        // Combine behaviours
        float vx_cmd = 0.0f;
        float vy_cmd = 0.0f;
        float vz_cmd = 0.0f;

        vx_cmd += ALIGNMENT_WEIGHT * align_vx;
        vy_cmd += ALIGNMENT_WEIGHT * align_vy;
        vz_cmd += ALIGNMENT_WEIGHT * align_vz;

        vx_cmd += COHESION_WEIGHT * coh_x;
        vy_cmd += COHESION_WEIGHT * coh_y;
        vz_cmd += COHESION_WEIGHT * coh_z;

        vx_cmd += SEPARATION_WEIGHT * sep_x;
        vy_cmd += SEPARATION_WEIGHT * sep_y;
        vz_cmd += SEPARATION_WEIGHT * sep_z;

        // Clamp to max speed
        float speed = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd + vz_cmd * vz_cmd);
        if (speed > static_cast<float>(MAX_VELOCITY_MM_S) && speed > 0.0f) {
            float scale = static_cast<float>(MAX_VELOCITY_MM_S) / speed;
            vx_cmd *= scale;
            vy_cmd *= scale;
            vz_cmd *= scale;
        }

        cmd.vx_cmd_mm_s = static_cast<int32_t>(vx_cmd);
        cmd.vy_cmd_mm_s = static_cast<int32_t>(vy_cmd);
        cmd.vz_cmd_mm_s = static_cast<int32_t>(vz_cmd);

        // Yaw alignment: face direction of motion in XY plane
        const float pi = 3.14159265358979323846f;

        float desired_yaw_rad = std::atan2(vy_cmd, vx_cmd);   // yaw in XY plane
        int32_t desired_yaw_cd = static_cast<int32_t>(desired_yaw_rad * 18000.0f / pi);

        int32_t err_cd = yaw_error_cd(self.yaw_cd, desired_yaw_cd);

        // Simple P controller on yaw
        const float yaw_kp = 5.0f; // tune later
        float yaw_rate_cmd = yaw_kp * static_cast<float>(err_cd); // cd/s

        // Clamp yaw rate
        const int32_t max_yaw_rate_cd_s = 18000; // 180 deg/s
        yaw_rate_cmd = clamp_float(
            yaw_rate_cmd,
            -static_cast<float>(max_yaw_rate_cd_s),
            static_cast<float>(max_yaw_rate_cd_s)
        );

        cmd.yaw_rate_cd_s = static_cast<int32_t>(yaw_rate_cmd);

        return cmd;
    }

    // --------------------------------------------------------
    // Metrics
    // --------------------------------------------------------

    FlockMetrics compute_flock_metrics(
        const LocalState& self,
        const std::vector<comms::NeighbourEntry>& neighbours
    )
    {
        FlockMetrics m{};

        if (neighbours.empty()) {
            m.min_separation_mm      = 0.0f;
            m.avg_distance_to_center = 0.0f;
            m.heading_alignment      = 0.0f;
            m.neighbour_count        = 0;
            return m;
        }

        float center_x = 0.0f;
        float center_y = 0.0f;
        float center_z = 0.0f;

        float min_sep = std::numeric_limits<float>::max();

        float self_vx = static_cast<float>(self.vx_mm_s);
        float self_vy = static_cast<float>(self.vy_mm_s);
        float self_vz = static_cast<float>(self.vz_mm_s);

        float self_speed = std::sqrt(self_vx * self_vx + self_vy * self_vy + self_vz * self_vz);
        float align_sum  = 0.0f;
        int   align_count = 0;

        for (const auto& n : neighbours) {
            float dx = static_cast<float>(n.x_mm) - static_cast<float>(self.x_mm);
            float dy = static_cast<float>(n.y_mm) - static_cast<float>(self.y_mm);
            float dz = static_cast<float>(n.z_mm) - static_cast<float>(self.z_mm);

            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            center_x += static_cast<float>(n.x_mm);
            center_y += static_cast<float>(n.y_mm);
            center_z += static_cast<float>(n.z_mm);

            if (dist < min_sep) {
                min_sep = dist;
            }

            // Heading alignment (cosine of angle between velocities)
            float nvx = static_cast<float>(n.vx_mm_s);
            float nvy = static_cast<float>(n.vy_mm_s);
            float nvz = static_cast<float>(n.vz_mm_s);

            float n_speed = std::sqrt(nvx * nvx + nvy * nvy + nvz * nvz);

            if (self_speed > 0.0f && n_speed > 0.0f) {
                float dot       = self_vx * nvx + self_vy * nvy + self_vz * nvz;
                float cos_theta = dot / (self_speed * n_speed);
                align_sum  += cos_theta;
                ++align_count;
            }
        }

        int N = static_cast<int>(neighbours.size());
        center_x /= static_cast<float>(N);
        center_y /= static_cast<float>(N);
        center_z /= static_cast<float>(N);

        float cdx = center_x - static_cast<float>(self.x_mm);
        float cdy = center_y - static_cast<float>(self.y_mm);
        float cdz = center_z - static_cast<float>(self.z_mm);
        float dist_to_center = std::sqrt(cdx * cdx + cdy * cdy + cdz * cdz);

        m.min_separation_mm      = (N > 0) ? min_sep : 0.0f;
        m.avg_distance_to_center = dist_to_center;
        m.neighbour_count        = N;
        m.heading_alignment      = (align_count > 0) ? (align_sum / align_count) : 0.0f;

        return m;
    }

} // namespace control
