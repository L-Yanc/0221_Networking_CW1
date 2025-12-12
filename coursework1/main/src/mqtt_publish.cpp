// mqtt_publish.cpp
//
// Simple MQTT client for publishing telemetry

#include "mqtt_publish.hpp"
#include <mqtt_client.h>
#include <esp_log.h>
#include <inttypes.h>

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
    // MQTT broker config: assuming broker on localhost:1883
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = "mqtt://localhost:1883";

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
