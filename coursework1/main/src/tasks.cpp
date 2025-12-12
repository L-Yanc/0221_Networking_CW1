// tasks.cpp
//
// Implements all 4 FreeRTOS tasks:
//  - physics_task:   integrates LocalState at 50 Hz
//  - flock_task:     computes ControlCmd from neighbours at 10 Hz
//  - radio_task:     builds and sends LoRa packets at ~4 Hz
//  - telemetry_task: logs state + metrics at ~2 Hz
//
// This module owns the single LocalState instance and a mutex to protect it.

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

#include <cstring>     // for std::memcpy
#include <inttypes.h>  // optional, if you ever use PRIu32 etc.


namespace tasks {

    static const char* TAG = "TASKS";

    // --------------------------------------------------------
    // Shared state and synchronisation
    // --------------------------------------------------------

    // Global local state for this drone
    static control::LocalState g_state;

    // Mutex protecting g_state
    static SemaphoreHandle_t s_state_mutex = nullptr;

    // Queue for commands from flock_task -> physics_task
    // Single-slot queue (latest command wins) is usually enough
    static QueueHandle_t s_cmd_queue = nullptr;

    // Node ID (derived from WiFi MAC once)
    static uint8_t s_node_id[6] = {0};
    static bool    s_node_id_initialised = false;

    // Helper RAII lock for g_state
    class StateLock {
    public:
        StateLock()  { xSemaphoreTake(s_state_mutex, portMAX_DELAY); }
        ~StateLock() { xSemaphoreGive(s_state_mutex); }
    };

    // Get a copy of the current state (used by flock, radio, telemetry)
    static control::LocalState get_state_snapshot()
    {
        StateLock lock;
        return g_state;
    }

    // Update the global state (only physics_task should call this)
    static void set_state(const control::LocalState& st)
    {
        StateLock lock;
        g_state = st;
    }

    // Ensure we have a node_id populated from WiFi MAC
    static void ensure_node_id_initialised()
    {
        if (s_node_id_initialised) {
            return;
        }

        uint8_t mac[6] = {0};
        esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_get_mac failed (%d), using fallback node_id", err);
            // Fallback: some arbitrary but non-zero ID
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

    // --------------------------------------------------------
    // Physics task
    // --------------------------------------------------------

    static void physics_task(void* arg)
    {
        (void)arg;
        ESP_LOGI(TAG, "physics_task started");

        const TickType_t period_ticks = pdMS_TO_TICKS(1000 / PHYSICS_HZ);
        TickType_t last_wake = xTaskGetTickCount();

        control::ControlCmd current_cmd{};  // last received command

        for (;;) {
            vTaskDelayUntil(&last_wake, period_ticks);

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

        for (;;) {
            vTaskDelayUntil(&last_wake, period_ticks);

            // Snapshot local state
            control::LocalState self = get_state_snapshot();

            // Snapshot neighbours from comms
            uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            auto neighbours = comms::get_neighbour_snapshot(now_ms);

            // Compute flocking control command
            control::ControlCmd cmd = control::compute_flock_command(self, neighbours);

            // Send to physics (queue length is 1, latest command wins)
            xQueueOverwrite(s_cmd_queue, &cmd);
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

        for (;;) {
            vTaskDelayUntil(&last_wake, period_ticks);

            uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

            // Cull stale neighbours periodically
            comms::cull_stale_neighbours(now_ms);

            // NOTE: Actual TX/RX is handled by radio_rx_task in comms.cpp
            // This task just manages neighbour table maintenance
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

        for (;;) {
            vTaskDelayUntil(&last_wake, period_ticks);

            uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

            // Snapshot local state and neighbours
            control::LocalState self = get_state_snapshot();
            auto neighbours = comms::get_neighbour_snapshot(now_ms);

            // Compute flocking metrics for logging / report
            control::FlockMetrics metrics = control::compute_flock_metrics(self, neighbours);

            // Log the existing telemetry (unchanged)
            ESP_LOGI(TAG,
                     "telemetry: x=%u y=%u z=%u v=(%d,%d,%d) neigh=%d minSep=%.1f align=%.2f",
                     (unsigned) self.x_mm,
                     (unsigned) self.y_mm,
                     (unsigned) self.z_mm,
                     (int) self.vx_mm_s,
                     (int) self.vy_mm_s,
                     (int) self.vz_mm_s,
                     (int) metrics.neighbour_count,
                     metrics.min_separation_mm,
                     metrics.heading_alignment);


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
            g_state.x_mm = 45000;   // 45m
            g_state.y_mm = 51000;   // 51m
            g_state.z_mm = 10000;   // 10m

            // Optional: initial orientation or velocity
            g_state.vx_mm_s = 0;
            g_state.vy_mm_s = 0;
            g_state.vz_mm_s = 0;
            g_state.yaw_cd  = 0;     // centidegrees if you care
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
