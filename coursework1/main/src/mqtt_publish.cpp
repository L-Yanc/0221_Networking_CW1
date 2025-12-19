#include "mqtt_publish.hpp"
#include <mqtt_client.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <inttypes.h>
#include <cstdio>
#include <cstring>
#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"
#include "config.hpp"

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGD(TAG, "Other MQTT event id:%" PRIi32, event_id);
            break;
    }
}

void mqtt_pub::init() {
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = "mqtt://broker.hivemq.com:1883";

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return;
    }

    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_CONNECTED, mqtt_event_handler, NULL);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, NULL);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_PUBLISHED, mqtt_event_handler, NULL);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ERROR, mqtt_event_handler, NULL);
    
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "MQTT client started");
}

void mqtt_pub::publish_telemetry(const char* json_payload) {
    if (mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized");
        return;
    }

    if (!mqtt_connected) {
        ESP_LOGW(TAG, "MQTT not connected");
        return;
    }

    int msg_id = esp_mqtt_client_publish(mqtt_client, "flocksim", json_payload, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Failed to publish MQTT message");
    }
}

bool mqtt_pub::is_connected() {
    return mqtt_connected;
}

bool mqtt_pub::publish_state(uint32_t x_mm, uint32_t y_mm, uint32_t z_mm,
                              int32_t vx_mm_s, int32_t vy_mm_s, int32_t vz_mm_s,
                              uint16_t yaw_cd, uint16_t seq_number)
{
    if (!mqtt_connected) {
        return false;
    }

    uint8_t mac_addr[6];
    esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);

    uint8_t struct_data[42];
    size_t idx = 0;

    struct_data[idx++] = PROTOCOL_VERSION;
    struct_data[idx++] = TEAM_ID;

    memcpy(&struct_data[idx], mac_addr, 6);
    idx += 6;

    struct_data[idx++] = (seq_number >> 8) & 0xFF;
    struct_data[idx++] = seq_number & 0xFF;
    idx += 2;

    uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
    uint16_t now_ms = static_cast<uint16_t>((esp_timer_get_time() / 1000ULL) % 1000);

    struct_data[idx++] = (now >> 24) & 0xFF;
    struct_data[idx++] = (now >> 16) & 0xFF;
    struct_data[idx++] = (now >> 8) & 0xFF;
    struct_data[idx++] = now & 0xFF;

    struct_data[idx++] = (now_ms >> 8) & 0xFF;
    struct_data[idx++] = now_ms & 0xFF;

    struct_data[idx++] = (x_mm >> 24) & 0xFF;
    struct_data[idx++] = (x_mm >> 16) & 0xFF;
    struct_data[idx++] = (x_mm >> 8) & 0xFF;
    struct_data[idx++] = x_mm & 0xFF;

    struct_data[idx++] = (y_mm >> 24) & 0xFF;
    struct_data[idx++] = (y_mm >> 16) & 0xFF;
    struct_data[idx++] = (y_mm >> 8) & 0xFF;
    struct_data[idx++] = y_mm & 0xFF;

    struct_data[idx++] = (z_mm >> 24) & 0xFF;
    struct_data[idx++] = (z_mm >> 16) & 0xFF;
    struct_data[idx++] = (z_mm >> 8) & 0xFF;
    struct_data[idx++] = z_mm & 0xFF;

    struct_data[idx++] = (vx_mm_s >> 24) & 0xFF;
    struct_data[idx++] = (vx_mm_s >> 16) & 0xFF;
    struct_data[idx++] = (vx_mm_s >> 8) & 0xFF;
    struct_data[idx++] = vx_mm_s & 0xFF;

    struct_data[idx++] = (vy_mm_s >> 24) & 0xFF;
    struct_data[idx++] = (vy_mm_s >> 16) & 0xFF;
    struct_data[idx++] = (vy_mm_s >> 8) & 0xFF;
    struct_data[idx++] = vy_mm_s & 0xFF;

    struct_data[idx++] = (vz_mm_s >> 24) & 0xFF;
    struct_data[idx++] = (vz_mm_s >> 16) & 0xFF;
    struct_data[idx++] = (vz_mm_s >> 8) & 0xFF;
    struct_data[idx++] = vz_mm_s & 0xFF;

    struct_data[idx++] = (yaw_cd >> 8) & 0xFF;
    struct_data[idx++] = yaw_cd & 0xFF;

    uint8_t cmac_tag[16] = {0};
    const mbedtls_cipher_info_t* cipher_info =
        mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);

    mbedtls_cipher_context_t ctx;
    mbedtls_cipher_init(&ctx);
    mbedtls_cipher_setup(&ctx, cipher_info);
    mbedtls_cipher_cmac_starts(&ctx, TEAM_KEY, 128);
    mbedtls_cipher_cmac_update(&ctx, struct_data, 42);
    mbedtls_cipher_cmac_finish(&ctx, cmac_tag);
    mbedtls_cipher_free(&ctx);

    uint8_t mac_tag[4];
    memcpy(mac_tag, cmac_tag + 12, 4);

    char json_buf[512];
    snprintf(json_buf, sizeof(json_buf),
        "{"
        "\"version\":%d,"
        "\"team_id\":%d,"
        "\"node_id\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"seq_number\":%u,"
        "\"ts_s\":%lu,"
        "\"ts_ms\":%u,"
        "\"x_mm\":%lu,"
        "\"y_mm\":%lu,"
        "\"z_mm\":%lu,"
        "\"vx_mm_s\":%ld,"
        "\"vy_mm_s\":%ld,"
        "\"vz_mm_s\":%ld,"
        "\"yaw_cd\":%u,"
        "\"mac_tag\":\"%02X%02X%02X%02X\""
        "}",
        PROTOCOL_VERSION,
        TEAM_ID,
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
        seq_number,
        (unsigned long)now,
        now_ms,
        (unsigned long)x_mm,
        (unsigned long)y_mm,
        (unsigned long)z_mm,
        (long)vx_mm_s,
        (long)vy_mm_s,
        (long)vz_mm_s,
        yaw_cd,
        mac_tag[0], mac_tag[1], mac_tag[2], mac_tag[3]
    );

    publish_telemetry(json_buf);
    return true;
}
