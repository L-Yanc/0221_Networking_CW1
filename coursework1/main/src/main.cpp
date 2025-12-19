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
#include "mqtt_publish.hpp"
#include "attacks.hpp"

static const char* TAG = "MAIN";

const uint8_t TEAM_KEY[16] = {
        0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B,
        0x0C, 0x0D, 0x0E, 0x0F,
    };

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
    mqtt_pub::init();
}

static void init_attacks()
{
    AttackMode mode = AttackMode::None;
    
    if (ENABLE_ATTACK_REPLAY) {
        mode = AttackMode::Replay;
    } else if (ENABLE_ATTACK_GHOST) {
        mode = AttackMode::Ghost;
    } else if (ENABLE_ATTACK_FALSEDATA) {
        mode = AttackMode::FalseData;
    } else if (ENABLE_ATTACK_FASTTX) {
        mode = AttackMode::FastTx;
    } else if (ENABLE_ATTACK_INVALIDCMAC) {
        mode = AttackMode::InvalidCmac;
    }
    
    attacks::set_attack_mode(mode);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting COMP0221 CW node...");

    init_nvs();

    ESP_ERROR_CHECK(wifi_connect());

    init_sntp();
    init_mqtt();
    init_attacks();

    comms::radio_init();
    comms::start_radio_rx();

    tasks::start_all_tasks();

    ESP_LOGI("MAIN", "CONFIG PHYSICS_HZ=%d FLOCK_HZ=%d RADIO_HZ=%d TELEMETRY_HZ=%d",
         PHYSICS_HZ, FLOCK_HZ, RADIO_HZ, TELEMETRY_HZ);

}
