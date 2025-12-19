#include "tasks.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "sntp_time.h"
}

#include "config.hpp"
#include "control.hpp"
#include "comms.hpp"
#include "attacks.hpp"
#include "mqtt_publish.hpp"

#include <cstring>
#include <inttypes.h>


namespace tasks {

    static inline int64_t ticks_to_us(TickType_t ticks)
    {
        return static_cast<int64_t>(ticks) * 1000000 / configTICK_RATE_HZ;
    }

    static const char* TAG = "TASKS";

    static control::LocalState g_state;

    static SemaphoreHandle_t s_state_mutex = nullptr;
    static QueueHandle_t s_cmd_queue = nullptr;

    static uint8_t s_node_id[6] = {0};
    static bool s_node_id_initialised = false;

    class StateLock {
    public:
        StateLock() { xSemaphoreTake(s_state_mutex, portMAX_DELAY); }
        ~StateLock() { xSemaphoreGive(s_state_mutex); }
    };

    static control::LocalState get_state_snapshot()
    {
        StateLock lock;
        return g_state;
    }

    static void set_state(const control::LocalState& st)
    {
        StateLock lock;
        g_state = st;
    }

    static void ensure_node_id_initialised()
    {
        if (s_node_id_initialised) {
            return;
        }

        uint8_t mac[6] = {0};
        esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_get_mac failed (%d), using fallback node_id", err);
            uint8_t fallback[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};
            std::memcpy(s_node_id, fallback, sizeof(s_node_id));
        } else {
            std::memcpy(s_node_id, mac, sizeof(s_node_id));
        }

        s_node_id_initialised = true;
        ESP_LOGI(TAG, "Node ID set to %02X:%02X:%02X:%02X:%02X:%02X",
                 s_node_id[0], s_node_id[1], s_node_id[2],
                 s_node_id[3], s_node_id[4], s_node_id[5]);
    }

    struct JitterTracker {
        int64_t prev_start_us = -1;

        int64_t update(int64_t start_us, int64_t period_us) {
            if (prev_start_us < 0) {
                prev_start_us = start_us;
                return 0; // no jitter on first sample
            }

            int64_t actual_period = start_us - prev_start_us;
            prev_start_us = start_us;
            return actual_period - period_us;
        }
    };

    // --------------------------------------------------------
    // Physics task
    // --------------------------------------------------------

    static void physics_task(void* arg)
    {
        (void)arg;
        ESP_LOGI(TAG, "physics_task started");

        const TickType_t period_ticks = pdMS_TO_TICKS(1000 / PHYSICS_HZ);
        TickType_t last_wake = xTaskGetTickCount();

        const TickType_t tick0 = last_wake;
        const int64_t us0 = esp_timer_get_time();

        static JitterTracker phys_jitter;

        control::ControlCmd current_cmd{};  // last received command

        for (;;) {

            vTaskDelayUntil(&last_wake, period_ticks);
            const int64_t t_exp_us = us0 + ticks_to_us(last_wake - tick0);
            const int64_t t_start_us = esp_timer_get_time();
            static uint32_t phys_print_count = 0;

            int64_t jitter_us = phys_jitter.update(
                t_start_us,
                (int64_t)(1000000LL / PHYSICS_HZ)
            );

            // Try to receive a new command (non-blocking)
            control::ControlCmd new_cmd;
            if (xQueueReceive(s_cmd_queue, &new_cmd, 0) == pdTRUE) {
                current_cmd = new_cmd;
            }

            // Integrate physics step
            control::LocalState st = get_state_snapshot();

            const float dt_s = 1.0f / static_cast<float>(PHYSICS_HZ);
            control::physics_step(st, current_cmd, dt_s);

            // Increment sequence number for telemetry / LoRa packets
            st.seq_number += 1;

            const int64_t t_end_us = esp_timer_get_time();

            phys_print_count++;

            if (phys_print_count >= PHYSICS_HZ) {
                phys_print_count = 0;
                ESP_LOGI(TAG,
                    "PHYSICS exp_us=%lld start_us=%lld end_us=%lld exec_us=%lld lat_us=%lld jitter_us=%lld p_us=%lld "
                    "seq=%u yaw_cd=%u v=(%d,%d,%d)",
                    (long long)t_exp_us, (long long)t_start_us, (long long)t_end_us,
                    (long long)(t_end_us - t_start_us),
                    (long long)(t_start_us - t_exp_us),
                    (long long)jitter_us,
                    (long long)(1000000LL / PHYSICS_HZ),
                    (unsigned)st.seq_number,
                    (unsigned)st.yaw_cd,
                    (int)st.vx_mm_s, (int)st.vy_mm_s, (int)st.vz_mm_s
                );
            }

            set_state(st);
        }
    }

    // --------------------------------------------------------
    // Flocking task
    // --------------------------------------------------------

    static void flock_task(void* arg)
    {
        (void)arg;
        ESP_LOGI(TAG, "flock_task started");

        const TickType_t period_ticks = pdMS_TO_TICKS(1000 / FLOCK_HZ);
        TickType_t last_wake = xTaskGetTickCount();
        static uint32_t flock_print_count = 0;

        const TickType_t tick0 = last_wake;
        const int64_t us0 = esp_timer_get_time();
        static JitterTracker flock_jitter;

        for (;;) {
            vTaskDelayUntil(&last_wake, period_ticks);

            const int64_t t_exp_us   = us0 + ticks_to_us(last_wake - tick0);
            const int64_t t_start_us = esp_timer_get_time();

            int64_t jitter_us = flock_jitter.update(
                t_start_us,
                (int64_t)(1000000LL / FLOCK_HZ)
            );

            // Snapshot local state
            control::LocalState self = get_state_snapshot();

            // Snapshot neighbours from comms
            uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            auto neighbours = comms::get_neighbour_snapshot(now_ms);

            uint32_t age_max_ms = 0;
            for (const auto& n : neighbours) {
                uint32_t age = now_ms - n.last_seen_ms;
                if (age > age_max_ms) age_max_ms = age;
            }

            // Compute flocking control command
            control::ControlCmd cmd = control::compute_flock_command(self, neighbours);

            // Send to physics (queue length is 1, latest command wins)
            xQueueOverwrite(s_cmd_queue, &cmd);

            const int64_t t_end_us = esp_timer_get_time();

            flock_print_count++;

            if (flock_print_count >= FLOCK_HZ) {
                flock_print_count = 0;
                ESP_LOGI(TAG,
                    "FLOCK exp_us=%lld start_us=%lld end_us=%lld exec_us=%lld lat_us=%lld jitter=%lld p_us=%lld "
                    "neigh=%d age_max_ms=%u cmd_v=(%d,%d,%d) cmd_yaw_rate=%d",
                    (long long)t_exp_us, (long long)t_start_us, (long long)t_end_us,
                    (long long)(t_end_us - t_start_us),
                    (long long)(t_start_us - t_exp_us),
                    (long long)jitter_us,
                    (long long)(1000000LL / FLOCK_HZ),
                    (int)neighbours.size(),
                    (unsigned)age_max_ms,
                    (int)cmd.vx_cmd_mm_s,
                    (int)cmd.vy_cmd_mm_s,
                    (int)cmd.vz_cmd_mm_s,
                    (int)cmd.yaw_rate_cd_s
                );
            }
        }
    }

    // --------------------------------------------------------
    // Radio task
    // --------------------------------------------------------

    static void radio_task(void* arg)
    {
        (void)arg;
        ESP_LOGI(TAG, "radio_task started");

        const TickType_t period_ticks = pdMS_TO_TICKS(1000 / RADIO_HZ);
        TickType_t last_wake = xTaskGetTickCount();

        static uint32_t radio_print_count = 0;
        static JitterTracker radio_jitter;

        const TickType_t tick0 = last_wake;
        const int64_t us0 = esp_timer_get_time();

        for (;;) {
            vTaskDelayUntil(&last_wake, period_ticks);

            const int64_t t_exp_us   = us0 + ticks_to_us(last_wake - tick0);
            const int64_t t_start_us = esp_timer_get_time();

            int64_t jitter_us = radio_jitter.update(
                t_start_us,
                (int64_t)(1000000LL / RADIO_HZ)
            );

            uint32_t now_ms = static_cast<uint32_t>((esp_timer_get_time() / 1000) & 0xFFFFFFFF);

            comms::cull_stale_neighbours(now_ms);

            auto neigh = comms::get_neighbour_snapshot(now_ms);
            uint32_t age_max_ms = 0;
            for (const auto& n : neigh) {
                uint32_t age = now_ms - n.last_seen_ms;
                if (age > age_max_ms) age_max_ms = age;
            }

            radio_print_count++;
            const int64_t t_end_us = esp_timer_get_time();
            if (radio_print_count >= RADIO_HZ) {
                radio_print_count = 0;
                ESP_LOGI(TAG,
                    "RADIO exp_us=%lld start_us=%lld end_us=%lld exec_us=%lld lat_us=%lld jitter=%lld p_us=%lld "
                    "neigh=%d age_max_ms=%u",
                    (long long)t_exp_us, (long long)t_start_us, (long long)t_end_us,
                    (long long)(t_end_us - t_start_us),
                    (long long)(t_start_us - t_exp_us),
                    (long long)jitter_us,
                    (long long)(1000000LL / RADIO_HZ),
                    (int)neigh.size(),
                    (unsigned)age_max_ms
                );
            }
        }
    }

    // --------------------------------------------------------
    // Telemetry task
    // --------------------------------------------------------

static void telemetry_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "telemetry_task started");

    const TickType_t period_ticks = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    
    static uint32_t tele_print_count = 0;
    static JitterTracker tele_jitter;

    const TickType_t tick0 = last_wake;
    const int64_t us0 = esp_timer_get_time();
    static uint32_t last_stab_ms = 0;

    static uint32_t last_energy_ms = 0;
    static uint32_t mqtt_ok_1s = 0;
    static uint32_t mqtt_fail_1s = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, period_ticks);

        const int64_t t_exp_us   = us0 + ticks_to_us(last_wake - tick0);
        const int64_t t_start_us = esp_timer_get_time();

        int64_t jitter_us = tele_jitter.update(
            t_start_us,
            (int64_t)(1000000LL / TELEMETRY_HZ)
        );

        uint32_t now_ms = (uint32_t)((esp_timer_get_time() / 1000) & 0xFFFFFFFF);

        control::LocalState self = get_state_snapshot();
        auto neighbours = comms::get_neighbour_snapshot(now_ms);

        int neigh_count = (int)neighbours.size();

        if ((uint32_t)(now_ms - last_stab_ms) >= 1000U) {
            last_stab_ms = now_ms;

            auto m = control::compute_flock_metrics(self, neighbours);

            // heading_alignment is cosine [-1, 1], scale to int for logging
            int heading_cos_x1000 = (int)(m.heading_alignment * 1000.0f);

            ESP_LOGI(TAG,
                "STAB neigh=%d dist_cent_mm=%d min_sep_mm=%d heading_cos_x1000=%d",
                (int)m.neighbour_count,
                (int)m.avg_distance_to_center,
                (int)m.min_separation_mm,
                heading_cos_x1000
            );
        }

        const int64_t t_mqtt0 = esp_timer_get_time();
        bool mqtt_ok = mqtt_pub::publish_state(self.x_mm, self.y_mm, self.z_mm,
                                               self.vx_mm_s, self.vy_mm_s, self.vz_mm_s,
                                               self.yaw_cd, self.seq_number);
    
        if (mqtt_ok) mqtt_ok_1s++;
        else mqtt_fail_1s++;

        const int64_t t_mqtt1 = esp_timer_get_time();
        const int64_t t_end_us = esp_timer_get_time();

        if ((uint32_t)(now_ms - last_energy_ms) >= 1000U) {
            last_energy_ms = now_ms;

            comms::CommsStats cs = comms::get_and_reset_stats();

            ESP_LOGI(TAG,
                "ENERGY lora_tx=%u lora_rx_ok=%u lora_rx_mac_fail=%u mqtt_ok=%u mqtt_fail=%u",
                (unsigned)cs.lora_tx,
                (unsigned)cs.lora_rx_ok,
                (unsigned)cs.lora_rx_mac_fail,
                (unsigned)mqtt_ok_1s,
                (unsigned)mqtt_fail_1s
            );

            mqtt_ok_1s = 0;
            mqtt_fail_1s = 0;
        }

        tele_print_count++;
        if (tele_print_count >= TELEMETRY_HZ) {
                tele_print_count = 0;
            ESP_LOGI(TAG,
                "TELE exp_us=%lld start_us=%lld end_us=%lld exec_us=%lld lat_us=%lld jitter=%lld p_us=%lld "
                "x=%u y=%u z=%u v=(%d,%d,%d) yaw_cd=%u seq=%u "
                "neigh=%d mqtt_ok=%d mqtt_exec_us=%lld",
                (long long)t_exp_us, (long long)t_start_us, (long long)t_end_us,
                (long long)(t_end_us - t_start_us),
                (long long)(t_start_us - t_exp_us),
                (long long)jitter_us,
                (long long)(1000000LL / TELEMETRY_HZ),
                (unsigned)self.x_mm, (unsigned)self.y_mm, (unsigned)self.z_mm,
                (int)self.vx_mm_s, (int)self.vy_mm_s, (int)self.vz_mm_s,
                (unsigned)self.yaw_cd,
                (unsigned)self.seq_number,
                neigh_count,
                (int)mqtt_ok,
                (long long)(t_mqtt1 - t_mqtt0)
            );
        }
    }
}

    // --------------------------------------------------------
    // Task startup
    // --------------------------------------------------------

    void start_all_tasks()
    {
        ESP_LOGI(TAG, "Creating tasks and queues");

        // Create mutex and queue
        s_state_mutex = xSemaphoreCreateMutex();
        configASSERT(s_state_mutex != nullptr);

        // Single element queue for ControlCmd (latest command only)
        s_cmd_queue = xQueueCreate(1, sizeof(control::ControlCmd));
        configASSERT(s_cmd_queue != nullptr);

        // Initialise state (LocalState default puts us roughly in the middle)
        {
            StateLock lock;
            g_state = control::LocalState{};

            // Set custom starting position in mm:
            g_state.x_mm = 30000;   // 30 m
            g_state.y_mm = 30000;   // 30 m
            g_state.z_mm = 30000;   // 30 m

            // Optional: initial orientation or velocity
            g_state.vx_mm_s = 0;
            g_state.vy_mm_s = 0;
            g_state.vz_mm_s = 0;
            g_state.yaw_cd  = 0;
        }

        // Create tasks
        BaseType_t ok;

        ok = xTaskCreate(
            physics_task,
            "physics",
            4096,
            nullptr,
            5,          // priority
            nullptr
        );
        configASSERT(ok == pdPASS);

        ok = xTaskCreate(
            flock_task,
            "flock",
            4096,
            nullptr,
            4,
            nullptr
        );
        configASSERT(ok == pdPASS);

        ok = xTaskCreate(
            radio_task,
            "radio",
            4096,
            nullptr,
            4,
            nullptr
        );
        configASSERT(ok == pdPASS);

        ok = xTaskCreate(
            telemetry_task,
            "telemetry",
            4096,
            nullptr,
            3,
            nullptr
        );
        configASSERT(ok == pdPASS);
    }

    // --------------------------------------------------------
    // Public API: get local state
    // --------------------------------------------------------

    control::LocalState get_local_state()
    {
        return get_state_snapshot();
    }

} // namespace tasks
