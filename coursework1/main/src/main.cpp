// main.cpp
//
// Entry point for the ESP-IDF application.
// - Initialise NVS
// - Connect to WiFi
// - Sync time via SNTP
// - Initialise LoRa radio + neighbour table
// - Start all application tasks

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "wifi_connect.h"
#include "sntp_time.h"
}

#include "config.hpp"
#include "comms.hpp"
#include "tasks.hpp"

static const char* TAG = "MAIN";


// -----------------------------------------------------
// NVS init
// -----------------------------------------------------
static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "NVS initialised");
}


// -----------------------------------------------------
// WiFi init using wifi_connect.c
// -----------------------------------------------------
static void init_wifi()
{
    ESP_LOGI(TAG, "Connecting to WiFi...");

    esp_err_t err = wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed: %d", err);
        // Up to you: we can continue without WiFi, but telemetry and SNTP will die
        // For now, just log and continue.
    } else {
        ESP_LOGI(TAG, "WiFi connected");
    }
}


// -----------------------------------------------------
// SNTP time sync using sntp_time.c
// -----------------------------------------------------
static void init_sntp()
{
    ESP_LOGI(TAG, "Syncing time via SNTP...");

    esp_err_t err = sync_time();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SNTP sync failed: %d", err);
        // Again, we can continue, but ts_s/ts_ms will be garbage.
        // CW spec prefers proper time, but code will still run.
    } else {
        ESP_LOGI(TAG, "SNTP time sync complete");
    }
}


// -----------------------------------------------------
// MQTT init
static void init_mqtt()
{
    ESP_LOGI(TAG, "MQTT init placeholder (not implemented)");
}


// -----------------------------------------------------
// app_main()
// -----------------------------------------------------
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting COMP0221 CW node...");
    ESP_LOGI("MAIN", "app_main starting up");

    // Core init
    init_nvs();
    init_wifi();
    init_sntp();
    init_mqtt();

    // Radio + neighbour table init
    comms::radio_init();
    comms::start_radio_rx();

    // Start all tasks (physics, flock, radio, telemetry)
    tasks::start_all_tasks();

    // app_main returns; FreeRTOS scheduler runs the tasks forever
}
