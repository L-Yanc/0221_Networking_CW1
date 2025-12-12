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

static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "NVS initialised");
}

// ----------------------------
// SNTP time sync
// ----------------------------
static void init_sntp()
{
    ESP_LOGI(TAG, "Syncing time via SNTP...");

    esp_err_t err = sync_time();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SNTP sync failed: %d", err);
    } else {
        ESP_LOGI(TAG, "SNTP time sync complete");
    }
}

// ----------------------------
// MQTT placeholder
// ----------------------------
static void init_mqtt()
{
    ESP_LOGI(TAG, "MQTT init placeholder");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting COMP0221 CW node...");

    init_nvs();

    // Call the real wifi_connect()
    ESP_ERROR_CHECK(wifi_connect());

    init_sntp();
    init_mqtt();

    comms::radio_init();
    comms::start_radio_rx();

    tasks::start_all_tasks();
}
