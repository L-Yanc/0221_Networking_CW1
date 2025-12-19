#include "comms.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"

#include "driver/gpio.h"
#include "esp_mac.h"
}

#include "RadioLib.h"
#include "EspHal.h"

#include <algorithm>
#include <cstring>
#include <vector>

#include "config.hpp"
#include "tasks.hpp"
#include "sntp_time.h"
#include "attacks.hpp"

namespace comms {

    static const char* TAG = "COMMS";

#ifndef LORA_SPI_SCK
    #define LORA_SPI_SCK   GPIO_NUM_5
#endif
#ifndef LORA_SPI_MISO
    #define LORA_SPI_MISO  GPIO_NUM_19
#endif
#ifndef LORA_SPI_MOSI
    #define LORA_SPI_MOSI  GPIO_NUM_27
#endif
#ifndef LORA_CS
    #define LORA_CS        GPIO_NUM_18
#endif
#ifndef LORA_DIO0
    #define LORA_DIO0      GPIO_NUM_26
#endif
#ifndef LORA_RST
    #define LORA_RST       GPIO_NUM_14
#endif
#ifndef LORA_BUSY
    #define LORA_BUSY GPIO_NUM_NC
#endif

    static EspHal s_hal(LORA_SPI_SCK, LORA_SPI_MISO, LORA_SPI_MOSI);
    static Module s_module(&s_hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_BUSY);
    static SX1276 s_radio(&s_module);

    static TaskHandle_t s_rx_task_handle = nullptr;

    static NeighbourEntry s_neighbours[MAX_NEIGHBOURS];
    static SemaphoreHandle_t s_neighbour_mutex = nullptr;

    static uint32_t s_lora_tx = 0;
    static uint32_t s_lora_rx_ok = 0;
    static uint32_t s_lora_rx_mac_fail = 0;

    class NeighbourLock {
    public:
        NeighbourLock()  { xSemaphoreTake(s_neighbour_mutex, portMAX_DELAY); }
        ~NeighbourLock() { xSemaphoreGive(s_neighbour_mutex); }
    };

    // Forward declaration so RX task can call it
    void handle_rx_bytes(const uint8_t* buf, size_t len);

    // --------------------------------------------------------
    // CMAC helpers
    // --------------------------------------------------------

    bool compute_cmac(const uint8_t* data, size_t len, uint8_t tag_out[MAC_TAG_LEN])
    {
        // full 16-byte CMAC buffer
        uint8_t full_tag[16] = {0};

        const mbedtls_cipher_info_t* cipher_info =
            mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
        if (cipher_info == nullptr) {
            ESP_LOGE(TAG, "compute_cmac: cipher_info_from_type failed");
            return false;
        }

        mbedtls_cipher_context_t ctx;
        mbedtls_cipher_init(&ctx);

        int ret = mbedtls_cipher_setup(&ctx, cipher_info);
        if (ret != 0) {
            ESP_LOGE(TAG, "compute_cmac: cipher_setup failed (%d)", ret);
            mbedtls_cipher_free(&ctx);
            return false;
        }

        // key = TEAM_KEY, 128 bits
        ret = mbedtls_cipher_cmac_starts(&ctx, TEAM_KEY, 128);
        if (ret != 0) {
            ESP_LOGE(TAG, "compute_cmac: cmac_starts failed (%d)", ret);
            mbedtls_cipher_free(&ctx);
            return false;
        }

        // message = first (PACKET_LEN_BYTES - MAC_TAG_LEN) bytes of packet
        ret = mbedtls_cipher_cmac_update(&ctx, data, len);
        if (ret != 0) {
            ESP_LOGE(TAG, "compute_cmac: cmac_update failed (%d)", ret);
            mbedtls_cipher_free(&ctx);
            return false;
        }

        ret = mbedtls_cipher_cmac_finish(&ctx, full_tag);
        mbedtls_cipher_free(&ctx);

        if (ret != 0) {
            ESP_LOGE(TAG, "compute_cmac: cmac_finish failed (%d)", ret);
            return false;
        }

        // Truncate to 4 bytes as per COMP0221 spec - use LAST 4 bytes, not first
        std::memcpy(tag_out, full_tag + 12, MAC_TAG_LEN);
        return true;
    }

    bool verify_cmac(const uint8_t* data, size_t len, const uint8_t tag[MAC_TAG_LEN])
    {
        uint8_t expected[MAC_TAG_LEN];

        if (!compute_cmac(data, len, expected)) {
            ESP_LOGE(TAG, "verify_cmac: compute_cmac failed");
            return false;
        }

        // constant-time-ish comparison
        return std::memcmp(expected, tag, MAC_TAG_LEN) == 0;
    }


    // --------------------------------------------------------
    // Low level radio helpers
    // --------------------------------------------------------

    static bool radio_send_bytes(const uint8_t* buf, size_t len)
    {
        // Ensure radio is in standby before transmitting
        s_radio.standby();
        
        int16_t state = s_radio.transmit(buf, len);
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGE(TAG, "radio_send_bytes: transmit failed (%d)", state);
            // Force radio back to RX mode on failure
            s_radio.startReceive();
            return false;
        }
        
        // Delay to allow TX to complete and radio to settle
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Explicitly return to RX mode after TX
        state = s_radio.startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGW(TAG, "radio_send_bytes: startReceive after TX failed (%d)", state);
            return false;
        }
        
        return true;
    }

    // --------------------------------------------------------
    // Helper: build a TX packet with real state
    // --------------------------------------------------------
    static void build_periodic_packet(LoraPacket& pkt)
    {
        static uint32_t s_seq = 0;
        static bool s_node_id_init = false;
        static uint8_t s_node_id[6] = {0};

        if (!s_node_id_init) {
            uint8_t mac[6];
            // Use Wi-Fi STA MAC as a simple unique node_id
            if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
                std::memcpy(s_node_id, mac, 6);
            }
            s_node_id_init = true;
        }

        std::memset(&pkt, 0, sizeof(LoraPacket));

        pkt.version   = PROTOCOL_VERSION;
        pkt.team_id   = TEAM_ID;
        std::memcpy(pkt.node_id, s_node_id, sizeof(pkt.node_id));
        pkt.seq_number = s_seq++;

        // Get real state from physics simulation
        control::LocalState state = tasks::get_local_state();
        pkt.x_mm    = state.x_mm;
        pkt.y_mm    = state.y_mm;
        pkt.z_mm    = state.z_mm;
        pkt.vx_mm_s = state.vx_mm_s;
        pkt.vy_mm_s = state.vy_mm_s;
        pkt.vz_mm_s = state.vz_mm_s;
        pkt.yaw_cd  = state.yaw_cd;

        // Fill timestamps
        uint32_t ts_s  = 0;
        uint16_t ts_ms = 0;
        get_current_unix_time(&ts_s, &ts_ms);
        pkt.ts_s  = ts_s;
        pkt.ts_ms = ts_ms;
    }

    
    static void radio_rx_task(void* arg)
    {
        static const char* TAG_RX = "COMMS_RX";

        uint8_t  buf[PACKET_LEN_BYTES];
        
        // Start in RX mode
        int16_t state = s_radio.startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGE(TAG_RX, "Failed to start RX mode (%d)", state);
        }

        uint32_t last_tx_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        while (true) {
            // Small delay to check for packets and avoid task watchdog
            vTaskDelay(pdMS_TO_TICKS(10));

            uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

            // --- TX every 6 seconds ---
            uint32_t elapsed = (now_ms >= last_tx_ms) ? (now_ms - last_tx_ms) : (UINT32_MAX - last_tx_ms + now_ms);
            if (elapsed >= 6000) {
                LoraPacket pkt{};
                build_periodic_packet(pkt);

                // Apply attacks before sending
                attacks::apply_attacks(attacks::get_attack_mode(), pkt, now_ms);

                // Temporarily leave RX mode to transmit
                if (send_packet(pkt)) {
                    ESP_LOGI(TAG_RX, "TX: periodic packet seq=%u sent at %u ms", (unsigned)pkt.seq_number, (unsigned)now_ms);
                    s_lora_tx++;
                } else {
                    ESP_LOGW(TAG_RX, "TX: send_packet failed");
                }

                last_tx_ms = now_ms;
                
                // Ensure back in RX mode after TX
                s_radio.startReceive();
            }

            // --- RX (GPIO polling) ---
            // Check if DIO0 pin indicates packet received
            if (gpio_get_level((gpio_num_t)LORA_DIO0) == 1) {
                memset(buf, 0, sizeof(buf));
                
                int rx_state = s_radio.readData(buf, sizeof(buf));

                if (rx_state == RADIOLIB_ERR_NONE) {
                    size_t pkt_len = s_radio.getPacketLength();
                    
                    if (pkt_len == PACKET_LEN_BYTES) {
                        // Got a full packet of expected length
                        comms::handle_rx_bytes(buf, PACKET_LEN_BYTES);
                    }
                }
                
                // Resume RX after reading
                s_radio.startReceive();
            }
        }
    }



    // This function should be called when a full packet is received
    void handle_rx_bytes(const uint8_t* buf, size_t len)
    {
        static const char* TAG_RX = "COMMS_RX";
        uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        LoraPacket pkt;
        if (!unpack_from_bytes(buf, len, pkt)) {
            return;
        }

        // Check version and team_id first (before CMAC verification)
        if (pkt.version != PROTOCOL_VERSION || pkt.team_id != TEAM_ID) {
            return;
        }

        // Verify AES-128-CMAC tag (mac_tag)
        uint8_t rx_mac_tag[MAC_TAG_LEN];
        std::memcpy(rx_mac_tag, pkt.mac_tag, MAC_TAG_LEN);
        
        // Zero the mac_tag field for CMAC verification
        std::memset(pkt.mac_tag, 0, sizeof(pkt.mac_tag));

        const size_t mac_input_len = PACKET_LEN_BYTES - MAC_TAG_LEN;

        if (!verify_cmac(reinterpret_cast<const uint8_t*>(&pkt),
                        mac_input_len,
                        rx_mac_tag)) {
            s_lora_rx_mac_fail++;
            return;
        }
        
        // Restore the mac_tag for neighbour update
        std::memcpy(pkt.mac_tag, rx_mac_tag, MAC_TAG_LEN);

        update_neighbour_from_packet(pkt, now_ms);
        s_lora_rx_ok++;
        
        // Log successful RX with what and from info
        ESP_LOGI(TAG_RX, "RX: from %02x:%02x:%02x:%02x:%02x:%02x seq=%u pos=(%lu,%lu,%lu)",
                 pkt.node_id[0], pkt.node_id[1], pkt.node_id[2],
                 pkt.node_id[3], pkt.node_id[4], pkt.node_id[5],
                 (unsigned)pkt.seq_number,
                 (unsigned long)pkt.x_mm, (unsigned long)pkt.y_mm, (unsigned long)pkt.z_mm);
    }

    // --------------------------------------------------------
    // Radio init
    // --------------------------------------------------------

    void radio_init()
    {
        ESP_LOGI(TAG, "Initialising radio and neighbour table");

        if (!s_neighbour_mutex) {
            s_neighbour_mutex = xSemaphoreCreateMutex();
        }

        {
            NeighbourLock lock;
            for (int i = 0; i < MAX_NEIGHBOURS; ++i) {
                s_neighbours[i].in_use = false;
            }
        }

        float freq_mhz = static_cast<float>(LORA_FREQ_HZ) / 1000000.0f;
        float bw_khz   = static_cast<float>(LORA_BW);

        int16_t state = s_radio.begin(freq_mhz,
                                      bw_khz,
                                      LORA_SF,
                                      LORA_CR,
                                      LORA_SYNCWORD,
                                      LORA_TX_POWER_DBM,
                                      LORA_PREAMBLE_LEN,
                                      0);
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGE(TAG, "radio_init: begin failed (%d)", state);
        } else {
            ESP_LOGI(TAG, "radio_init: RadioLib begin OK (%.3f MHz, %.1f kHz, SF%d)",
                     freq_mhz, bw_khz, LORA_SF);
        }
    }

    void start_radio_rx()
    {
        if (!s_rx_task_handle) {
            BaseType_t ok = xTaskCreate(
                radio_rx_task,
                "lora_rx_task",
                4096,
                nullptr,
                5,
                &s_rx_task_handle
            );
            if (ok != pdPASS) {
                ESP_LOGE(TAG, "start_radio_rx: failed to create RX task");
            } else {
                ESP_LOGI(TAG, "start_radio_rx: RX task created");
            }
        } else {
            vTaskResume(s_rx_task_handle);
            ESP_LOGI(TAG, "start_radio_rx: RX task resumed");
        }
    }

    // --------------------------------------------------------
    // Packet serialisation
    // --------------------------------------------------------

    void pack_to_bytes(const LoraPacket& pkt, uint8_t* buf_out)
    {
        std::memcpy(buf_out, &pkt, sizeof(LoraPacket));
    }

    bool unpack_from_bytes(const uint8_t* buf, size_t len, LoraPacket& pkt_out)
    {
        if (len != PACKET_LEN_BYTES) {
            return false;
        }
        std::memcpy(&pkt_out, buf, sizeof(LoraPacket));
        return true;
    }

    bool send_packet(const LoraPacket& pkt_in)
    {
        LoraPacket pkt = pkt_in;
        
        // Ensure mac_tag is zeroed before computing CMAC
        // (CMAC must be computed over all fields except mac_tag)
        std::memset(pkt.mac_tag, 0, sizeof(pkt.mac_tag));

        const size_t mac_input_len = PACKET_LEN_BYTES - MAC_TAG_LEN;
        if (!compute_cmac(reinterpret_cast<const uint8_t*>(&pkt),
                          mac_input_len,
                          pkt.mac_tag)) {
            ESP_LOGE(TAG, "send_packet: CMAC compute failed");
            return false;
        }

        uint8_t buf[PACKET_LEN_BYTES];
        pack_to_bytes(pkt, buf);

        return radio_send_bytes(buf, sizeof(buf));
    }

    // --------------------------------------------------------
    // Neighbour table
    // --------------------------------------------------------

    void update_neighbour_from_packet(const LoraPacket& pkt, uint32_t now_ms)
    {
        NeighbourLock lock;

        int match_index = -1;
        int free_index  = -1;

        // Try to find existing neighbour
        for (int i = 0; i < MAX_NEIGHBOURS; ++i) {
            if (s_neighbours[i].in_use) {
                if (std::memcmp(s_neighbours[i].node_id,
                                pkt.node_id,
                                sizeof(pkt.node_id)) == 0) {
                    match_index = i;
                    break;
                }
            } else if (free_index < 0) {
                free_index = i;
            }
        }

        int idx = match_index;
        if (idx < 0) {
            // Not found, add new entry
            if (free_index < 0) {
                idx = 0;  // Table full, overwrite first entry
            } else {
                idx = free_index;
            }
            s_neighbours[idx].in_use = true;
            std::memcpy(s_neighbours[idx].node_id, pkt.node_id, 6);
        }

        // Check if this is an old sequence number (replay check)
        if (s_neighbours[idx].in_use &&
            pkt.seq_number < s_neighbours[idx].seq_number) {
            // Old packet, ignore
            return;
        }

        // Update neighbour state
        s_neighbours[idx].seq_number   = pkt.seq_number;
        s_neighbours[idx].x_mm         = pkt.x_mm;
        s_neighbours[idx].y_mm         = pkt.y_mm;
        s_neighbours[idx].z_mm         = pkt.z_mm;
        s_neighbours[idx].vx_mm_s      = pkt.vx_mm_s;
        s_neighbours[idx].vy_mm_s      = pkt.vy_mm_s;
        s_neighbours[idx].vz_mm_s      = pkt.vz_mm_s;
        s_neighbours[idx].yaw_cd       = pkt.yaw_cd;
        s_neighbours[idx].last_seen_ms = now_ms;
    }

    void cull_stale_neighbours(uint32_t now_ms)
    {
        NeighbourLock lock;

        for (int i = 0; i < MAX_NEIGHBOURS; ++i) {
            if (!s_neighbours[i].in_use) {
                continue;
            }
            uint32_t age = now_ms - s_neighbours[i].last_seen_ms;
            if (age > NEIGHBOUR_TIMEOUT_MS) {
                s_neighbours[i].in_use = false;
            }
        }
    }

    std::vector<NeighbourEntry> get_neighbour_snapshot(uint32_t now_ms)
    {
        (void)now_ms;

        NeighbourLock lock;

        std::vector<NeighbourEntry> result;
        result.reserve(MAX_NEIGHBOURS);

        for (int i = 0; i < MAX_NEIGHBOURS; ++i) {
            if (s_neighbours[i].in_use) {
                result.push_back(s_neighbours[i]);
            }
        }
        return result;
    }

    CommsStats get_and_reset_stats()
    {
        CommsStats out{
            .lora_tx = s_lora_tx,
            .lora_rx_ok = s_lora_rx_ok,
            .lora_rx_mac_fail = s_lora_rx_mac_fail
        };

        s_lora_tx = 0;
        s_lora_rx_ok = 0;
        s_lora_rx_mac_fail = 0;

        return out;
    }


} // namespace comms
